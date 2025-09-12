// Minimal stub for rtc::scoped_refptr to satisfy checks.h template.
#ifndef API_SCOPED_REFPTR_H_
#define API_SCOPED_REFPTR_H_

namespace rtc {

template <typename T>
class scoped_refptr {
 public:
  scoped_refptr() : ptr_(nullptr) {}
  explicit scoped_refptr(T* p) : ptr_(p) {}
  T* get() const { return ptr_; }
  T* operator->() const { return ptr_; }
  T& operator*() const { return *ptr_; }
 private:
  T* ptr_;
};

}  // namespace rtc

#endif  // API_SCOPED_REFPTR_H_

