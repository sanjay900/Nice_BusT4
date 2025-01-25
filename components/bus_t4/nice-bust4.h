/*
  Nice BusT4
  Обмен данными по UART на скорости 19200 8n1
  Перед пакетом с данными отправляется break длительностью 519us (10 бит)
  Содержимое пакета, которое удалось понять, описано в структуре packet_cmd_body_t

 

  Для Oview к адресу всегда прибавляется 80.
  Адрес контроллера ворот без изменений.


Подключение

BusT4                       ESP32

Стенка устройства        Rx Tx GND
9  7  5  3  1
10 8  6  4  2
место для кабеля
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V




Из мануала nice_dmbm_integration_protocol.pdf

• ADR: это адрес сети NICE, где находятся устройства, которыми вы хотите управлять. Это может быть значение от 1 до 63 (от 1 до 3F).
Это значение должно быть в HEX. Если адресатом является модуль интеграции на DIN-BAR, это значение равно 0 (adr = 0), если адресат
является интеллектуальным двигателем, это значение равно 1 (adr = 1).
• EPT: это адрес двигателя Nice, входящего в сетевой ADR. Это может быть значение от 1 до 127. Это значение должно быть в HEX.
• CMD: это команда, которую вы хотите отправить по назначению (ADR, EPT).
• PRF: команда установки профиля.
• FNC: это функция, которую вы хотите отправить по назначению (ADR, EPT).
• EVT: это событие, которое отправляется в пункт назначения (ADR, EPT).
*/

#pragma once

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#include <vector>
#include <string>
#include <queue>

#include "consts.h"

static const uint32_t BAUD_BREAK = 9200; /* бодрэйт для длинного импульса перед пакетом */
static const uint32_t BAUD_WORK = 19200; /* рабочий бодрэйт */
static const uint8_t START_CODE = 0x55; /*стартовый байт пакета */

static const float CLOSED_POSITION_THRESHOLD = 0.007;
static const uint32_t POSITION_UPDATE_INTERVAL = 500;

namespace esphome {
namespace bus_t4 {

/* Команда, которая должна быть выполнена.   
11-й байт пакета CMD
Используется в запросах и ответах */
enum control_cmd : uint8_t { 
  SBS = 0x01,    /* Step by Step */
  STOP = 0x02,   /* Stop */
  OPEN = 0x03,   /* Open */
  CLOSE = 0x04,  /* Close */
  P_OPN1 = 0x05, /* Partial opening 1 - частичное открывание, режим калитки */
  P_OPN2 = 0x06, /* Partial opening 2 */
  P_OPN3 = 0x07, /* Partial opening 3 */
  RSP = 0x19, /* ответ интерфейса, подтверждающий получение команды  */
  EVT = 0x29, /* ответ интерфейса, отправляющий запрошенную информацию */
 
  P_OPN4 = 0x0b, /* Partial opening 4 - Коллективно */
  P_OPN5 = 0x0c, /* Partial opening 5 - Приоритет пошагово */
  P_OPN6 = 0x0d, /* Partial opening 6 - Открыть и блокировать */
  UNLK_OPN = 0x19, /* Разблокировать и открыть */
  CLS_LOCK = 0x0E, /* Закрыть и блокировать */
  UNLCK_CLS = 0x1A, /*  Разблокировать и Закрыть */
  LOCK = 0x0F, /* Блокировать*/
  UNLOCK = 0x10, /* Разблокировать */
  LIGHT_TIMER = 0x11, /* Таймер освещения */
  LIGHT_SW = 0x12, /* Освещение вкл/выкл */
  HOST_SBS = 0x13, /* Ведущий SBS */
  HOST_OPN = 0x14, /* Ведущий открыть */
  HOST_CLS = 0x15, /* Ведущий закрыть */
  SLAVE_SBS = 0x16, /*  Ведомый SBS */
  SLAVE_OPN = 0x17, /* Ведомый открыть */
  SLAVE_CLS = 0x18, /* Ведомый закрыть */
  AUTO_ON = 0x1B, /* Автооткрывание активно */
  AUTO_OFF = 0x1C, /* Автооткрывание неактивно	  */
  
};
	

class NiceBusT4 : public cover::Cover, public Component, public uart::UARTDevice {
  public:

    // настройки привода
    bool autocls_flag; // Автозакрывание - L1
    bool photocls_flag; // Закрыть после фото - L2
    bool alwayscls_flag; // Всегда закрывать - L3
    bool init_ok = false; //  определение привода при включении
    bool is_walky = false; // для walky отличается команда запроса положения
    bool is_robus = false; // для robus не нужно переодически запрашивать позицию
		
    void setup() override;
    void loop() override;
    void dump_config() override; // для вывода в лог информации об оборудовнии

    void send_raw_cmd(std::string data);
    void send_cmd(uint8_t data) {this->tx_buffer_.push(gen_control_cmd(data));}	
    void send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command,  std::string next_data, bool data_on, std::string data_command); // длинная команда
    void set_mcu(std::string command, std::string data_command); // команда контроллеру мотора
		

    void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }
    
 /*   void set_update_interval(uint32_t update_interval) {  // интервал получения статуса привода
      this->update_interval_ = update_interval;
    }*/

    cover::CoverTraits get_traits() override;

  protected:
    void control(const cover::CoverCall &call) override;
    void send_command_(const uint8_t *data, uint8_t len);
    void request_position(void);  // Запрос условного текущего положения привода
    void update_position(uint16_t newpos);  // Обновление текущего положения привода

    uint32_t last_position_time{0};  // Время последнего обновления текущего положения
    uint32_t update_interval_{500};
    uint32_t last_update_{0};
    uint32_t last_uart_byte_{0};

    cover::CoverOperation last_published_op;  // Последние опубликованные состояние и положение
    float last_published_pos{-1};

    void publish_state_if_changed(void);

    uint8_t position_hook_type{IGNORE};  // Флаг и позиция установки заданного положения привода
    uint16_t position_hook_value;

    uint8_t class_gate_ = 0x55; // 0x01 sliding, 0x02 sectional, 0x03 swing, 0x04 barrier, 0x05 up-and-over
//    uint8_t last_init_command_;
	
    bool init_cu_flag = false;
    bool init_oxi_flag = false;

	
    // переменные для uart
    uint8_t _uart_nr;
    uart_t* _uart = nullptr;
    uint16_t _max_opn = 0;  // максимальная позиция энкодера или таймера
    uint16_t _pos_opn = 2048;  // позиция открытия энкодера или таймера, не для всех приводов.
    uint16_t _pos_cls = 0;  // позиция закрытия энкодера или таймера, не для всех приводов
    uint16_t _pos_usl = 0;  // условная текущая позиция энкодера или таймера, не для всех приводов	
    // настройки заголовка формируемого пакета
    uint8_t addr_from[2] = {0x00, 0x66}; //от кого пакет, адрес bust4 шлюза
    uint8_t addr_to[2]; // = 0x00ff;	 // кому пакет, адрес контроллера привода, которым управляем
    uint8_t addr_oxi[2]; // = 0x000a;	 // адрес приемника

    std::vector<uint8_t> raw_cmd_prepare (std::string data);             // подготовка введенных пользователем данных для возможности отправки	
	
    // генерация inf команд
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len);	 // все поля
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, 0x00, {0x00}, 0 );} // для команд без данных
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, std::vector<uint8_t> data){
	    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, next_data, data, data.size());} // для команд c данными
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data){
	    return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);} // для команд с адресом и без данных 	
    	    
    // генерация cmd команд
    std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);
	
    void init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device );
    void send_array_cmd (std::vector<uint8_t> data);	
    void send_array_cmd (const uint8_t *data, size_t len);


    void parse_status_packet (const std::vector<uint8_t> &data); // разбираем пакет статуса
    
    void handle_char_(uint8_t c);                                         // обработчик полученного байта
    void handle_datapoint_(const uint8_t *buffer, size_t len);          // обработчик полученных данных
    bool validate_message_();                                         // функция проверки полученного сообщения

    std::vector<uint8_t> rx_message_;                          // здесь побайтно накапливается принятое сообщение
    std::queue<std::vector<uint8_t>> tx_buffer_;             // очередь команд для отправки	
    bool ready_to_tx_{true};	                           // флаг возможности отправлять команды
	
    std::vector<uint8_t> manufacturer_ = {0x55, 0x55};  // при инициализации неизвестный производитель
    std::vector<uint8_t> product_;
    std::vector<uint8_t> hardware_;
    std::vector<uint8_t> firmware_;
    std::vector<uint8_t> description_;
    std::vector<uint8_t> oxi_product;
    std::vector<uint8_t> oxi_hardware;
    std::vector<uint8_t> oxi_firmware;
    std::vector<uint8_t> oxi_description;
};
}
}  // namespace esphome

