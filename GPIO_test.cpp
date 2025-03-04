#include <JetsonGPIO.h>
#include <iostream>
#include <unistd.h>

#define GPIO_PIN 35

int main() {
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(GPIO_PIN, GPIO::OUT);
    
    GPIO::NumberingModes mode = GPIO::getmode();


    for (int i = 0; i < 5; i++) {
        GPIO::output(GPIO_PIN, GPIO::HIGH);
        std::cout << "LED ON" << std::endl;
        sleep(1);

        GPIO::output(GPIO_PIN, GPIO::LOW);
        std::cout << "LED OFF" << std::endl;
        sleep(1);
    }

    GPIO::cleanup();
    return 0;
}
