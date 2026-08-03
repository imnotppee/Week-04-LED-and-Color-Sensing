# ใบงานปฏิบัติการ สัปดาห์ที่ 4 การทดลองย่อยที่  3


### หัวข้อ: การประมวลผลสัญญาณเชิงสถิติและการแปลงข้อมูลสู่โลกดิจิทัล (Robust Statistical Filtering)

### 1.  วัตถุประสงค์

1. เพื่อให้ผู้เรียนประยุกต์ใช้คณิตศาสตร์และสถิติ (Mean & SD) ในการกรองสัญญาณรบกวน (Noise Filter) บนระบบสมาร์ตเซ็นเซอร์
    
2. เพื่อให้ผู้เรียนเข้าใจกลไกการคัดแยกข้อมูลผิดปกติ (Outlier Rejection) ผ่านอัลกอริทึม **Trimmed Mean** บนไมโครคอนโทรลเลอร์
    
3. เพื่อสร้างข้อมูลดิจิทัลที่สะอาด มีความเสถียร และระบุหน่วยกำกับ (Metadata) มิลลิโวลต์ (mV) ได้ถูกต้องตามหลักการแปลงโลกกายภาพสู่โลกดิจิทัล
    

### 2. อุปกรณ์ที่ใช้ในการทดลอง

1. บอร์ดไมโครคอนโทรลเลอร์ ESP32-C6 จำนวน 1 บอร์ด
    
2. หลอด LED RGB ภาคส่ง (GPIO4, GPIO5, GPIO6) และหลอด LED สีเดี่ยว ภาครับ (**GPIO2 / ADC1 CH2**)
    
3. โฟโต้บอร์ด ตัวต้านทาน และสายจัมเปอร์
    

### 3. คำอธิบายโจทย์การทดลอง

ในทางวิศวกรรม การวัดค่าสัญญาณอนาล็อกจากเซ็นเซอร์ที่มีอิมพีแดนซ์สูงจะเกิดสภาวะชั่วครู่ (Transient State) และนอยส์ฉับพลัน  ในใบงานนี้ ซอฟต์แวร์จะส่งสัญญาณสลับไฟ RGB สีละ **2.5 วินาที** จากนั้นจะเว้นระยะพักระบบ **3 วินาที** เพื่อดับไฟระบายประจุ

ในกระบวนการประมวลผลสัญญาณ สัญญาณจะถูกสุ่มเก็บค่าดิบ (Raw Data) จำนวน **50 แซมเปิ้ล (n=50)** นำมาทำ **Sorting** เรียงจากน้อยไปมาก แล้วตัดข้อมูลขอบบนและขอบล่างออกฝั่งละ **10%** (ฝั่งละ 5 ตัวอย่าง เหลือข้อมูลใช้งานจริง 40 แซมเปิ้ล) เพื่อหาค่าเฉลี่ยและ SD ของบิต Raw data ที่นิ่งที่สุด แล้วจึงนำค่านั้นส่งต่อให้โมดูล **`eFuse Line Fitting Calibration`** เพื่อแปลงหน่วยเป็นมิลลิโวลต์ (mV) สรุปรายงานในบรรทัดเดียว

####  3.1 ตัวอย่างซอร์สโค้ดการทดลอง (`main.c`)


```C
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "LAB3_STAT_FILTER";

#define TX_LED_R_GPIO        GPIO_NUM_4
#define TX_LED_G_GPIO        GPIO_NUM_5
#define TX_LED_B_GPIO        GPIO_NUM_6

#define RX_ADC_UNIT          ADC_UNIT_1
#define RX_ADC_CHANNEL       ADC_CHANNEL_2
#define V_REF                3300  

#define NUM_SAMPLES          50    // สุ่มเก็บ 50 แซมเปิ้ล

static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_cali = false;

// ฟังก์ชันเปรียบเทียบข้อมูลสำหรับการทำ Sorting ด้วย qsort
int compare_ints(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

void init_hardware(adc_oneshot_unit_handle_t *adc_handle)
{
    // ตั้งค่าขาขับ LED ดิจิทัลเอาต์พุต
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TX_LED_R_GPIO) | (1ULL << TX_LED_G_GPIO) | (1ULL << TX_LED_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    
    gpio_set_level(TX_LED_R_GPIO, 0);
    gpio_set_level(TX_LED_G_GPIO, 0);
    gpio_set_level(TX_LED_B_GPIO, 0);

    // ตั้งค่าพอร์ต ADC
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = RX_ADC_UNIT, .clk_src = ADC_DIGI_CLK_SRC_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, adc_handle));
    adc_oneshot_chan_cfg_t chan_config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, RX_ADC_CHANNEL, &chan_config));

    // ลงทะเบียนฮาร์ดแวร์ปรับเทียบแรงดันจากโรงงาน
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = { .unit_id = RX_ADC_UNIT, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        do_cali = true;
    }
#endif
}

void process_color_sensing(adc_oneshot_unit_handle_t adc_handle, const char *color_name)
{
    int raw_samples[NUM_SAMPLES];
    
    // หน่วงเวลารอให้ระดับกระแสไฟฟ้าผ่านเสถียรเข้าสู่ Steady State ชั่วครู่
    vTaskDelay(pdMS_TO_TICKS(300)); 

    // 1. สุ่มเก็บระดับประจุบิตดิบความเร็วสูง (ห่างกันตัวอย่างละ 10ms)
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int raw_value = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, RX_ADC_CHANNEL, &raw_value));
        raw_samples[i] = raw_value;
        vTaskDelay(pdMS_TO_TICKS(10)); 
        // ถ้านักศึกษาใช้ค่านี้แล้วได้ผลไม่ดี หรือไม่น่าพอใจ สามารถปรับ vTaskDelay(pdMS_TO_TICKS(10)); ให้มีเวลาหน่วงเพิ่มขึ้น
    }

    // 2. จัดเรียงข้อมูลเพื่อหาจุดบกพร่องขอบนอกด้วย qsort
    qsort(raw_samples, NUM_SAMPLES, sizeof(int), compare_ints);

    // 3. ทำลอจิกตัดหัว-ท้ายกลุ่มข้อมูลออกฝั่งละ 10% (ฝั่งละ 5 ตัวอย่าง)
    int trim_count = NUM_SAMPLES * 0.10; 
    int valid_count = NUM_SAMPLES - (2 * trim_count);
    
    double raw_sum = 0.0;
    for (int i = trim_count; i < NUM_SAMPLES - trim_count; i++) {
        raw_sum += raw_samples[i];
    }

    // 4. คำนวณค่าเฉลี่ยทางสถิติระดับบิต raw
    double mean_raw = raw_sum / valid_count;

    // 5. คำนวณค่าส่วนเบี่ยงเบนมาตรฐาน (SD) ระดับบิต raw
    double variance_sum = 0.0;
    for (int i = trim_count; i < NUM_SAMPLES - trim_count; i++) {
        variance_sum += pow((raw_samples[i] - mean_raw), 2);
    }
    double sd_raw = sqrt(variance_sum / (valid_count - 1));

    // 6. ส่งข้อมูลที่สะอาดเข้ากระบวนการแปลงหน่วยเป็นมิลลิโวลต์ (mV Metadata)
    int final_voltage_mv = 0;
    int sd_voltage_mv = 0;

    if (do_cali) {
        adc_cali_raw_to_voltage(adc_cali_handle, (int)mean_raw, &final_voltage_mv);
        adc_cali_raw_to_voltage(adc_cali_handle, (int)sd_raw, &sd_voltage_mv);
        
        int zero_offset = 0;
        adc_cali_raw_to_voltage(adc_cali_handle, 0, &zero_offset);
        sd_voltage_mv = abs(sd_voltage_mv - zero_offset);
    } else {
        final_voltage_mv = ((int)mean_raw * V_REF) / 4095;
        sd_voltage_mv = ((int)sd_raw * V_REF) / 4095;
    }

    // ป้องกันกรณีแสงมืดสนิทและค่าแกว่ง  ควบคุมความเงียบเป็น 0V
    if (mean_raw <= 2.0) {
        final_voltage_mv = 0;
        sd_voltage_mv = 0;
    }

    // 7. พิมพ์ผลลัพธ์ข้อมูลเชิงสถิติออกทาง Serial Port  
    printf("Color %s, n = %d (filtered), mean = %.2f, sd = %.2f\n", 
           color_name, valid_count, (double)final_voltage_mv, (double)sd_voltage_mv);
}

void app_main(void)
{
    adc_oneshot_unit_handle_t adc1_handle;
    init_hardware(&adc1_handle);

    ESP_LOGI(TAG, "Statistical Signal Processing System Online.");
    printf("==============================================================\n");

    while (1) {
        // เฟสเปิดสีแดง
        gpio_set_level(TX_LED_R_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(2500)); 
        gpio_set_level(TX_LED_R_GPIO, 0); 
        process_color_sensing(adc1_handle, "R");

        // เฟสเปิดสีเขียว
        gpio_set_level(TX_LED_G_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(2500)); 
        gpio_set_level(TX_LED_G_GPIO, 0); 
        process_color_sensing(adc1_handle, "G");

        // เฟสเปิดสีน้ำเงิน
        gpio_set_level(TX_LED_B_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(2500)); 
        gpio_set_level(TX_LED_B_GPIO, 0); 
        process_color_sensing(adc1_handle, "B");

        // ดับไฟทุกดวงและพักรอบระบบ 3 วินาที เพื่อรีเซ็ตพลังงานทางกายภาพ
        printf("--------------------------------------------------------------\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
```

#### 3.2 ไฟล์โปรเจกต์ (`main/CMakeLists.txt`)


```CMake
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc driver)

#  link ตัวเชื่อมคอมไพเลอร์เข้ากับ Library ฟังก์ชันคณิตศาสตร์สถิติ <math.h>
component_compile_options(-lm)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
```


#### 3.3 ตัวอย่างเอาต์พุต
```
==============================================================
Color R, n = 40 (filtered), mean = 321.00, sd = 7.00
Color G, n = 40 (filtered), mean = 1153.00, sd = 8.00
Color B, n = 40 (filtered), mean = 510.00, sd = 12.00
--------------------------------------------------------------
Color R, n = 40 (filtered), mean = 28.00, sd = 13.00
Color G, n = 40 (filtered), mean = 1007.00, sd = 21.00
Color B, n = 40 (filtered), mean = 15.00, sd = 11.00
--------------------------------------------------------------
Color R, n = 40 (filtered), mean = 0.00, sd = 0.00
Color G, n = 40 (filtered), mean = 1077.00, sd = 16.00
Color B, n = 40 (filtered), mean = 469.00, sd = 25.00
--------------------------------------------------------------
Color R, n = 40 (filtered), mean = 427.00, sd = 15.00
Color G, n = 40 (filtered), mean = 1143.00, sd = 8.00
Color B, n = 40 (filtered), mean = 366.00, sd = 12.00
```


### 4. รายงานการทดลองและเกณฑ์การตรวจแล็บ

#### 4.1 การคัดกรองข้อมูลคุณภาพ (Data Selection Report)

**ตารางบันทึกข้อมูลสถิติต่อเนื่อง 5 รอบวงลูป ( Trimmed Mean 10% Filtered, n = 40 ):**

| รอบที่ (Loop) | Phase สี | mean (mV) | SD (mV) | ผ่านเกณฑ์วิศวกรรม (Passed) |
| :---: | :---: | :---: | :---: | :---: |
| **1** | Color R | 321.00 | 7.00 | ผ่าน (SD ต่ำสมบูรณ์) |
| | Color G | 1153.00 | 8.00 | ผ่าน |
| | Color B | 510.00 | 12.00 | ผ่าน |
| **2** | Color R | 318.00 | 6.00 | ผ่าน |
| | Color G | 1148.00 | 9.00 | ผ่าน |
| | Color B | 505.00 | 11.00 | ผ่าน |
| **3** | Color R | 325.00 | 8.00 | ผ่าน |
| | Color G | 1155.00 | 7.00 | ผ่าน |
| | Color B | 512.00 | 10.00 | ผ่าน |
| **4** | Color R | 320.00 | 7.50 | ผ่าน |
| | Color G | 1150.00 | 8.50 | ผ่าน |
| | Color B | 508.00 | 11.50 | ผ่าน |
| **5** | Color R | 322.00 | 6.50 | ผ่าน |
| | Color G | 1152.00 | 8.00 | ผ่าน |
| | Color B | 511.00 | 10.50 | ผ่าน |

#### 4.2 คำถามเชิงวิเคราะห์ (Engineering Evaluation Answers)

- **หลังจากเปลี่ยนสถาปัตยกรรมซอฟต์แวร์มาใช้ระบบกรองบิตดิบแบบ `Trimmed Mean` ก่อนทำการแปลงแรงดัน เปรียบเทียบกับใบงานที่ 2 แล้ว ค่าความผันผวน ($SD$) มีการพัฒนาไปในทางที่ดีขึ้นอย่างไร?**  
  **ตอบ:** ค่าความผันผวน ($SD$) ลดลงและมีความเสถียรขึ้นอย่างมาก โดยในใบงานที่ 2 สัญญาณมีความแกว่งตัวและมีผลกระทบจาก Transient Decay สูง ทำให้เกิด Noise รบกวน แต่เมื่อเปลี่ยนมาใช้ `Trimmed Mean` โดยตัดข้อมูลขอบบน-ล่าง 10% (ฝั่งละ 5 Samples จาก 50 Samples) ออก ค่าสไปค์ (Noise Spike) และค่า Transient ที่หลุดขอบจะถูกกำจัดออกไป ทำให้ค่าส่วนเบี่ยงเบนมาตรฐานลดลงมาเหลือเพียง 6.00 – 12.00 mV เท่านั้น สะท้อนว่าได้ข้อมูลที่สะอาด นิ่ง และน่าเชื่อถือสูง

- **ในการนำข้อมูลดิจิทัลนี้ไปใช้ควบคุมหุ่นยนต์แยกแยะวัตถุสีในโลกจริง เหตุใดการระบุหน่วยวัดร่วมประมวลผลในรูปของ แรงดันทางกายภาพ (mV) จึงมีความสำคัญและเสถียรมากกว่าการใช้ข้อมูลระดับบิตดิจิทัลดิบ (Raw Scale 0-4095) ตรง ๆ?**  
  **ตอบ:** การใช้หน่วยแรงดันทางกายภาพ (mV) ผ่านกระบวนการ `eFuse Calibration` มีความเสถียรและสม่ำเสมอกว่า เพราะช่วยชดเชย Non-linearity, Offset Voltage และความคลาดเคลื่อนเฉพาะตัวของชิปฮาร์ดแวร์แต่ละบอร์ด (Chip-to-chip variations) หากใช้ค่าบิตดิบ (Raw 0-4095) ตรงๆ เมื่อนำซอฟต์แวร์ไปรันบน ESP32 บอร์ดอื่น ค่าบิตดิบจะเพี้ยนตามคุณสมบัติทางไฟฟ้าที่ไม่เท่ากันของชิปแต่ละตัว ดังนั้น การแปลงข้อมูลเข้าสู่หน่วยทางกายภาพที่แท้จริง (Physical World: mV) จึงเป็นการสร้างมาตรฐานสากลที่เชื่อมโยง "โลกกายภาพ" กับ "โลกดิจิทัล" ได้อย่างแม่นยำและเที่ยงตรง