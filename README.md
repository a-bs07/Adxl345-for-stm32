# Adxl345-for-stm32
This is a driver of ADXL345 accelerometer for stm32 microcontrollers .HAL-Compatible.

HOW TO USE IT?* 

1.Configure the I2C Instance:
   - This code uses **I2C1**, with **PA10** (SDA) and **PA9** (SCL).  
   - If you need a different I2C instance, update it in the configuration and modify the declaration in `adxl_port.c`:  
     extern I2C_HandleTypeDef hi2cX;
   - *(Note: If you're using I2C1, no changes are needed.)*  

2. Configure the UART Instance for Logging:
   - The default UART instance used is **UART2**(PA2 and PA15 as TX and RX).  
   - To use a different UART instance:  
     - Update the configuration settings.  
     - Modify `extern UART_HandleTypeDef huart2;` in `adxl_app.c`.  
     - Update `main.c` at **line 62** to ensure `printf` works with the selected instance.  
   - *(Note: If you're using UART2, no changes are needed.)*  

3. Adjust Tap Detection Parameters:  
   - Modify the **latent value, window value, threshold value,** and **duration value** as per your requirements.  

4. Select Tap Detection Method:
   - By default, tap detection is implemented through **continuous polling** of flags.  
   - If you prefer **interrupt-based** detection:  
     - Enable **interrupt settings** in the configurator.  
     - No additional code changes are required—the necessary APIs are already implemented and tested.
5. You can take reference from the examples provided in the Test_codes folder.
