//Author: sdavi


#ifndef SOFTWAREPWMTIMER_H
#define SOFTWAREPWMTIMER_H

class SoftwarePWM; //Fwd decl

class SoftwarePWMTimer
{
public:
    SoftwarePWMTimer() noexcept;
    void EnableChannel(SoftwarePWM *sChannel) noexcept;
    void DisableChannel(SoftwarePWM *sChannel) noexcept;
    void Interrupt() noexcept;

private:

    
};

extern SoftwarePWMTimer softwarePWMTimer;

#endif
