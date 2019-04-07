#ifndef __ISR_WRAPPER__
#define __ISR_WRAPPER__


#include <functional>


namespace earth_rover_firmware {

  template<int irq>
  class ISRWrapper {
    private:
      bool our_handler_active_;
      static std::function<void()> isr_;
    public:
      ISRWrapper();
      ~ISRWrapper();
      bool attachISR(std::function<void()> isr, int mode);
      bool detachISR();
      static void ISR();
  };


  template<int irq>
  std::function<void()> ISRWrapper<irq>::isr_{};


  template<int irq>
  ISRWrapper<irq>::ISRWrapper():
    our_handler_active_{false}
  {
    ;
  }


  template<int irq>
  ISRWrapper<irq>::~ISRWrapper() {
    detachISR();  // detachISR function checks whether our handler is active or not.
  }


  template<int irq>
  bool ISRWrapper<irq>::attachISR(std::function<void()> isr, int mode) {
    bool attached_isr{false};
    if(!isr_) {
      our_handler_active_ = true;
      isr_ = isr;
      attachInterrupt(irq, &ISRWrapper<irq>::ISR, mode);
      attached_isr = true;
    }
    // Return true (success) if our new handler is active or false (failure) if another handler was active.
    return attached_isr;
  }


  template<int irq>
  bool ISRWrapper<irq>::detachISR() {
    if(our_handler_active_) {
      detachInterrupt(irq);
      isr_ = std::function<void()>();
      our_handler_active_ = false;
    }
    // Return true (succes) if no handler is active or false (failure) is another object's handler is active.
    return !isr_;
  }


  template<int irq>
  void ISRWrapper<irq>::ISR() {
    if(isr_) {
      isr_();
    }
  }

}


#endif