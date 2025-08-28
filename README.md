STM32 Geliştirme Çalışmaları
Bu repo, TAAC Staj Çalışmaları kapsamında geliştirilen çeşitli STM32 tabanlı gömülü sistem projelerini içermektedir. Her proje, belirli bir donanım özelliğini veya haberleşme protokolünü kullanmayı hedeflemektedir.

📂 Projeler:


🔘 Butona Göre Süre Ayarlama (ButonSuresi)

•	Buton ile belirli sürelerin ölçülmesi ve LED/PWM kontrolü.

•	Amaç: GPIO okuma, debounce ve zamanlama yönetimi.

🎚️ Center Aligned PWM (CenterAlignedPwm)

•	Timer ile center-aligned PWM üretimi.

•	Uygulama: Motor sürücü veya hassas sinyal üretimi.

⚡ DAC ve ADC Konfigürasyonu (DACveADCKONFIGURASYON)

•	DAC çıkışı → ADC girişi test uygulaması.

•	Analog sinyal üretimi ve ölçümü.

⏱️ Interrupt Yönetimi (Interrupts)

•	Harici ve timer tabanlı kesme (IRQ) yönetimi örnekleri.

•	Donanım tetiklemeleri ve ISR kullanımı.

💡 LED Yakma (LedYakma1)

•	Temel GPIO çıkışı.

•	Staj başlangıç seviyesi uygulama.

🔗 Master-Slave Haberleşme (MasterSlave)

•	I2C / SPI protokolleri ile master-slave haberleşme.

•	Çoklu cihaz kontrol örneği.

📡 PWM Uygulamaları (PWM)

•	Farklı frekans ve duty cycle ile PWM üretimi.

•	LED dimming ve motor sürme testleri.

⚙️ Rotary Encoder (RotaryTimerEncoder)

•	Encoder sinyallerinin Timer ile okunması.

•	Sayısal dönüş ölçümü (ör. potansiyometre, motor shaft).

🎛️ Timer Input Capture (TimerInputCapturePwm)

•	Timer ile input capture kullanımı.

•	PWM sinyallerinin ölçümü (frekans, duty cycle).

🔄 UART Haberleşme State Machine (UartHaberlesmeStateMachine)

•	UART protokolü ile haberleşme.

•	State machine yaklaşımıyla kararlı veri akışı.

🌐 Web Denemeleri (WEBDENEME2 ve WebDenemeProjesi)

•	STM32’nin web entegrasyonu / veri aktarımı üzerine testler.

•	IoT tabanlı denemeler.

🛠️ Kullanılan Teknolojiler

•	Mikrodenetleyici: STM32 serisi

•	IDE: STM32CubeIDE

•	Dil: C (HAL Kütüphaneleri ile)

•	Haberleşme Protokolleri: UART, I2C, SPI

•	Zamanlama: Timer, PWM, Input Capture, Encoder

•	Analog İşlemler: ADC, DAC,DMA
