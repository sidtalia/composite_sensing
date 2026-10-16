#pragma once

#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace composite_sensing {

/**
 * Cross-process color publish phase sync (no ROS).
 * Master writes last color-publish ROS time (ns); followers read it to lock stride phase.
 */
struct ColorSyncShmLayout {
  std::atomic<uint64_t> stamp_ns{0};
  std::atomic<uint64_t> seq{0};
};

inline std::string color_sync_shm_path(const std::string & master_cam)
{
  // POSIX shm_open name: leading slash, alnum/_ only.
  std::string name = "/hound_csync_";
  for (char c : master_cam) {
    name.push_back((std::isalnum(static_cast<unsigned char>(c)) || c == '_') ? c : '_');
  }
  return name;
}

class ColorSyncShm
{
public:
  ColorSyncShm() = default;
  ~ColorSyncShm() { close(); }

  ColorSyncShm(const ColorSyncShm &) = delete;
  ColorSyncShm & operator=(const ColorSyncShm &) = delete;

  bool open(const std::string & shm_name, bool create)
  {
    close();
    name_ = shm_name;
    // Always O_CREAT so followers that start before the master still get a handle;
    // only the master truncates/initializes.
    fd_ = ::shm_open(name_.c_str(), O_RDWR | O_CREAT, 0666);
    if (fd_ < 0) {
      return false;
    }
    if (create) {
      if (::ftruncate(fd_, static_cast<off_t>(sizeof(ColorSyncShmLayout))) != 0) {
        close();
        return false;
      }
    } else {
      struct stat st {};
      if (::fstat(fd_, &st) != 0 ||
        st.st_size < static_cast<off_t>(sizeof(ColorSyncShmLayout)))
      {
        close();
        return false;  // master has not initialized yet — caller should retry
      }
    }
    void * p = ::mmap(
      nullptr, sizeof(ColorSyncShmLayout), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (p == MAP_FAILED) {
      close();
      return false;
    }
    layout_ = static_cast<ColorSyncShmLayout *>(p);
    if (create) {
      layout_->stamp_ns.store(0, std::memory_order_relaxed);
      layout_->seq.store(0, std::memory_order_relaxed);
    }
    return true;
  }

  void close()
  {
    if (layout_ != nullptr) {
      ::munmap(layout_, sizeof(ColorSyncShmLayout));
      layout_ = nullptr;
    }
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void write_stamp_ns(uint64_t stamp_ns)
  {
    if (layout_ == nullptr) {
      return;
    }
    layout_->stamp_ns.store(stamp_ns, std::memory_order_release);
    layout_->seq.fetch_add(1, std::memory_order_acq_rel);
  }

  bool read_stamp_ns(uint64_t * stamp_ns, uint64_t * seq = nullptr) const
  {
    if (layout_ == nullptr || stamp_ns == nullptr) {
      return false;
    }
    const uint64_t s = layout_->stamp_ns.load(std::memory_order_acquire);
    if (s == 0) {
      return false;
    }
    *stamp_ns = s;
    if (seq != nullptr) {
      *seq = layout_->seq.load(std::memory_order_acquire);
    }
    return true;
  }

  bool ok() const { return layout_ != nullptr; }

private:
  std::string name_;
  int fd_{-1};
  ColorSyncShmLayout * layout_{nullptr};
};

}  // namespace composite_sensing
