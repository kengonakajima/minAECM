// Minimal stub for rtc::scoped_refptr used by rtc_base/checks.h
#ifndef API_SCOPED_REFPTR_H_
#define API_SCOPED_REFPTR_H_

namespace rtc {

template <typename T>
class scoped_refptr {
 public:
  scoped_refptr() : p_(nullptr) {}
  explicit scoped_refptr(T* p) : p_(p) {}
  T* get() const { return p_; }
 private:
  T* p_;
};

}  // namespace rtc

#endif  // API_SCOPED_REFPTR_H_

