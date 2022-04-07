package spotpuppy

import "time"

type UPSTimer struct {
	lastUpdate time.Time
	UPS        float64
}

func (u *UPSTimer) WaitForNext() {
	for time.Since(u.lastUpdate).Seconds() < 1.0/u.UPS {
	}
	u.lastUpdate = time.Now()
}

func NewUPSTimer(ups float64) *UPSTimer {
	return &UPSTimer{
		lastUpdate: time.Now(),
		UPS:        ups,
	}
}
