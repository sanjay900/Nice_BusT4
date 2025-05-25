#include "nice-bust4.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}

/*
  command dumps overview

  SBS               55 0c 00 ff 00 66 01 05 9D 01 82 01 64 E6 0c
  STOP              55 0c 00 ff 00 66 01 05 9D 01 82 02 64 E5 0c
  OPEN              55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c
  CLOSE             55 0c 00 ff 00 66 01 05 9D 01 82 04 64 E3 0c
  PARENTAL OPEN 1   55 0c 00 ff 00 66 01 05 9D 01 82 05 64 E2 0c
  PARENTAL OPEN 2   55 0c 00 ff 00 66 01 05 9D 01 82 06 64 E1 0c



*/

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;
  if (call.get_stop()) {
    send_cmd(STOP);

  } else if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);

      } else if (newpos == COVER_CLOSED) {
        if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);

      } else { // Arbitrary position
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Required actuator position: %d", position_hook_value);
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::setup() {


  this->current_operation = COVER_OPERATION_IDLE;
}

void NiceBusT4::loop() {

    if ((millis() - this->last_update_) > 10000) {    // every 10 seconds
// If the drive is not detected the first time, we will try again later.
        std::vector<uint8_t> unknown = {0x55, 0x55};
        if (this->init_ok == false) {
          this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
          this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //product request
        }
        
        else if (this->class_gate_ == 0x55) {
		init_device(this->addr_to[0], this->addr_to[1], 0x04);  
	//        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //product request
	}
        else if (this->manufacturer_ == unknown)  {
                init_device(this->addr_to[0], this->addr_to[1], 0x04);  
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //product request
		
        }
        this->last_update_ = millis();
    }  // if  every minute

	
  // allow sending every 100 ms
    uint32_t now = millis();
  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  } 


  while (this->parent_->available() > 0) {

    uint8_t c = this->read();                // read byte
    this->handle_char_(c);                                     // send byte for processing
    this->last_uart_byte_ = now;
  } //while

  if (this->ready_to_tx_) {   // if it's okay to send
    if (!this->tx_buffer_.empty()) {  // if there's anything to send
      this->send_array_cmd(this->tx_buffer_.front()); // send the first team in line
      this->tx_buffer_.pop();
      this->ready_to_tx_ = false;
    }
  }

  // Polling of the current actuator position
  if (!is_robus) {
  
  now = millis();
  if (init_ok && (current_operation != COVER_OPERATION_IDLE) && (now - last_position_time > POSITION_UPDATE_INTERVAL)) {
  	last_position_time = now;
    request_position();
  } 
  } // not robus

} //loop


void NiceBusT4::handle_char_(uint8_t c) {
  if (this->rx_message_.size() == 0) {
    this->rx_message_.push_back(0);
  }
  this->rx_message_.push_back(c);                      // throw a byte at the end of the received message
  if (!this->validate_message_()) {                    // check the resulting message
    this->rx_message_.clear();                         // If the check fails, the message is garbage and should be deleted.
  }
}


bool NiceBusT4::validate_message_() {                    // message validation
  uint32_t at = this->rx_message_.size() - 1;       // number of the last byte received
  uint8_t *data = &this->rx_message_[0];               // pointer to the first byte of the message
  uint8_t new_byte = data[at];                      // last byte received

  // Byte 0: HEADER1 (всегда 0x00)
  if (at == 0)
    return new_byte == 0x00;
  // Byte 1: HEADER2 (всегда 0x55)
  if (at == 1)
    return new_byte == START_CODE;

  // Byte 2: packet_size - number of bytes further + 1
  // Verification is not carried out

  if (at == 2)
    return true;
  uint8_t packet_size = data[2];
  uint8_t length = (packet_size + 3); // the length of the expected message is clear


  // Byte 3: Series (row) to whom package
  // Verification is not carried out
  //  uint8_t command = data[3];
  if (at == 3)
    return true;

  // Byte 4: Address to whom the package
  // Byte 5: Series (row) from whom the package is from
  // Byte 6: The address from whom the package is from
  // Byte 7: CMD or INF message type
  // Byte 8: The number of bytes next minus the two CRC bytes at the end.

  if (at <= 8)
    // There is no verification
    return true;

  uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);

  // Byte 9: crc1 = XOR (Byte 3 : Byte 8) XOR of the previous six bytes
  if (at == 9)
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Received invalid message checksum 1 %02X!=%02X", data[9], crc1);
      return false;
    }
  // Byte 10:
  // ...

  // waiting for all the package data to come in.
  if (at  < length)
    return true;

  // calculate crc2
  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < length - 1; i++) {
    crc2 = (crc2 ^ data[i]);
  }

  if (data[length - 1] != crc2 ) {
    ESP_LOGW(TAG, "Received invalid message checksum 2 %02X!=%02X", data[length - 1], crc2);
    return false;
  }

  // Byte Last: packet_size
  //  if (at  ==  length) {
  if (data[length] != packet_size ) {
    ESP_LOGW(TAG, "Received invalid message size %02X!=%02X", data[length], packet_size);
    return false;
  }

  // If it reached here - the correct message has been received and is in the rx_message_ buffer

  // Delete 0x00 at the beginning of the message
  rx_message_.erase(rx_message_.begin());

  // to log the packet
  std::string pretty_cmd = format_hex_pretty(rx_message_);
  ESP_LOGI(TAG,  "Packet received: %S ", pretty_cmd.c_str() );

  // we're doing something about the message here
  parse_status_packet(rx_message_);



  // return false to reset the rx buffer to zero
  return false;

}


// parse the received packets
void NiceBusT4::parse_status_packet (const std::vector<uint8_t> &data) {
  if ((data[1] == 0x0d) && (data[13] == 0xFD)) { // mistake
    ESP_LOGE(TAG,  "The command is not available for this device" );
  }


  if (((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // if evt
  //  ESP_LOGD(TAG, "An EVT packet of data has been received. Last cell %d ", data[12]);
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string str(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    ESP_LOGI(TAG,  "Data string: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "HEX data %S ", pretty_data.c_str() );
    // EVT data packet received, let's start parsing it

    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == GET - 0x80) && (data[13] == NOERR)) { // I am interested in completed responses to GET requests that come without errors from the drive
      ESP_LOGI(TAG,  "Response received %X ", data[10] );
      switch (data[10]) { // cmd_submnu
        case TYPE_M:
          //           ESP_LOGI(TAG,  "Gate type %X",  data[14]);
          switch (data[14]) { //14
            case SLIDING:
              this->class_gate_ = SLIDING;
              //        ESP_LOGD(TAG, "Gate type: Sliding %#X ", data[14]);
              break;
            case SECTIONAL:
              this->class_gate_ = SECTIONAL;
              //        ESP_LOGD(TAG, "Gate type: Sectional %#X ", data[14]);
              break;
            case SWING:
              this->class_gate_ = SWING;
              //        ESP_LOGD(TAG, "Gate type: Swinging %#X ", data[14]);
              break;
            case BARRIER:
              this->class_gate_ = BARRIER;
              //        ESP_LOGD(TAG, "Gate type: Barrier %#X ", data[14]);
              break;
            case UPANDOVER:
              this->class_gate_ = UPANDOVER;
              //        ESP_LOGD(TAG, "Gate type: Lift-and-turn %#X ", data[14]);
              break;
          }  // switch 14
          break; //  TYPE_M
        case INF_IO: // response to a request for the position of the sliding gate limit switch
          switch (data[16]) { //16
            case 0x00:
              ESP_LOGI(TAG, "  The fuse didn't work ");
              break; // 0x00
            case 0x01:
              ESP_LOGI(TAG, "  Closing limit switch ");
              this->position = COVER_CLOSED;
              break; //  0x01
            case 0x02:
              ESP_LOGI(TAG, "  Release lever ");
              this->position = COVER_OPEN;
              break; // 0x02

          }  // switch 16
          this->publish_state_if_changed();  // publish the status

          break; //  INF_IO


        //position of maximum encoder opening, opening, closing

        case MAX_OPN:
          if (is_walky) {
            this->_max_opn = data[15];
            this->_pos_opn = data[15];
          }
          else {  
            this->_max_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Maximum encoder position: %d", this->_max_opn);
          break;

        case POS_MIN:
          this->_pos_cls = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Closed gate position: %d", this->_pos_cls);
          break;

        case POS_MAX:
          if (((data[14] << 8) + data[15])>0x00) { // if the actuator response contains data on the opening position
          this->_pos_opn = (data[14] << 8) + data[15];}
          ESP_LOGI(TAG, "Open gate position: %d", this->_pos_opn);
          break;

        case CUR_POS:
          if (is_walky)
            update_position(data[15]);
          else
            update_position((data[14] << 8) + data[15]);
          break;

        case INF_STATUS:
          switch (data[14]) {
            case OPENED:
              ESP_LOGI(TAG, "  The gate is open");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_OPEN;
              break;
            case CLOSED:
              ESP_LOGI(TAG, "  The gate is closed");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_CLOSED;
              break;
            case 0x01:
              ESP_LOGI(TAG, "  The gate's stopped");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case 0x00:
              ESP_LOGI(TAG, "  The status of the gate is unknown");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
             case 0x0b:
              ESP_LOGI(TAG, "  The search for provisions has been made");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
              case STA_OPENING:
              ESP_LOGI(TAG, "  It's opening");
              this->current_operation = COVER_OPERATION_OPENING;
              break;
              case STA_CLOSING:
              ESP_LOGI(TAG, "  Closing in progress");
              this->current_operation = COVER_OPERATION_CLOSING;
              break;
          }  // switch
          this->publish_state_if_changed();  // публикуем состояние
          break;

          //      default: // cmd_mnu
        case AUTOCLS:
          this->autocls_flag = data[14];
	  ESP_LOGCONFIG(TAG, "  Auto-closing - L1: %S ", autocls_flag ? "Yes" : "No");	
          break;
          
        case PH_CLS_ON:
          this->photocls_flag = data[14];
          break;  
          
        case ALW_CLS_ON:
          this->alwayscls_flag = data[14];
          break;  
          
      } // switch cmd_submnu
    } // if completed responses to GET requests that came without errors from the drive

     if ((data[6] == INF) &&  (data[11] == GET - 0x81) && (data[13] == NOERR)) { // are interested in incomplete responses to GET requests that arrive without errors from all the
	ESP_LOGI(TAG,  "Received incomplete response to request %X, continued at offset %X", data[10], data[12] );
	     // repeat the command with a new offset
	tx_buffer_.push(gen_inf_cmd(data[4], data[5], data[9], data[10], GET, data[12]));
     
     } // Incomplete responses to GET requests that arrive without errors from the drive

	  
    
    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == SET - 0x80) && (data[13] == NOERR)) { // I am interested in responses to SET requests that have been received without errors from the drive    
      switch (data[10]) { // cmd_submnu
        case AUTOCLS:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, AUTOCLS, GET)); // Auto-closing
          break;
          
        case PH_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, PH_CLS_ON, GET)); // Close after Photo
          break;  
          
        case ALW_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, ALW_CLS_ON, GET)); // Always close
          break;  
      }// switch cmd_submnu
    }// if responses to SET requests received without errors from the drive

    if ((data[6] == INF) && (data[9] == FOR_ALL)  && ((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // interested in FOR_ALL responses to GET requests that arrive without errors

      switch (data[10]) {
        case MAN:
          //       ESP_LOGCONFIG(TAG, "  Manufacturer: %S ", str.c_str());
          this->manufacturer_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case PRD:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // if the packet from the receiver
//            ESP_LOGCONFIG(TAG, "  Receiver: %S ", str.c_str());
            this->oxi_product.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } // if the packet from the receiver
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // if the packet is from the drive controller
//            ESP_LOGCONFIG(TAG, "  Drive: %S ", str.c_str());
            this->product_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
            std::vector<uint8_t> wla1 = {0x57,0x4C,0x41,0x31,0x00,0x06,0x57}; // to understand that the Walky drive
            std::vector<uint8_t> ROBUSHSR10 = {0x52,0x4F,0x42,0x55,0x53,0x48,0x53,0x52,0x31,0x30,0x00}; // to understand that the ROBUSHSR10 actuator
            if (this->product_ == wla1) { 
              this->is_walky = true;
         //     ESP_LOGCONFIG(TAG, "  WALKY drive!: %S ", str.c_str());
                                        }
            if (this->product_ == ROBUSHSR10) { 
              this->is_robus = true;
          //    ESP_LOGCONFIG(TAG, "  ROBUS drive!: %S ", str.c_str());
                                        }		  
		  
          }
          break;
        case HWR:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // if the packet from the receiver
            this->oxi_hardware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // if the packet is from the drive controller         
          this->hardware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case FRM:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // if the packet from the receiver
            this->oxi_firmware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // if the packet is from the drive controller       
            this->firmware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case DSC:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // if the packet from the receiver
            this->oxi_description.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // if the packet is from the drive controller          
            this->description_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case WHO:
          if (data[12] == 0x01) {
            if (data[14] == 0x04) { // drive
              this->addr_to[0] = data[4];
              this->addr_to[1] = data[5];
              this->init_ok = true;
     //         init_device(data[4], data[5], data[14]);
            }
            else if (data[14] == 0x0A) { // receiver
              this->addr_oxi[0] = data[4];
              this->addr_oxi[1] = data[5];
              init_device(data[4], data[5], data[14]);
            }
          }
          break;
      }  // switch

    }  // if  FOR_ALL responses to GET requests that arrive without errors

    if ((data[9] == 0x0A) &&  (data[10] == 0x25) &&  (data[11] == 0x01) &&  (data[12] == 0x0A) &&  (data[13] == NOERR)) { //  packets from the receiver with information about the list of remotes that have arrived without errors
      ESP_LOGCONFIG(TAG, "Remote number: %X%X%X%X%X, command: %X, button: %X, mode: %X, press counter: %d", vec_data[5], vec_data[4], vec_data[3], vec_data[2], vec_data[8] / 0x10, vec_data[5] / 0x10, vec_data[7] + 0x01, vec_data[6]);
    }  // if

    if ((data[9] == 0x0A) &&  (data[10] == 0x26) &&  (data[11] == 0x41) &&  (data[12] == 0x08) &&  (data[13] == NOERR)) { //  пакеты от приемника с информацией о считанной кнопке пульта
      ESP_LOGCONFIG(TAG, "Button %X, remote control number: %X%X%X%X%X%X", vec_data[0] / 0x10, vec_data[0] % 0x10, vec_data[1], vec_data[2], vec_data[3]);
    }  // if

  } //  if evt



  //else if ((data[14] == NOERR) && (data[1] > 0x0d)) {  // otherwise Responce packet - acknowledgement of received command
  else if (data[1] > 0x0d) {  // otherwise Responce packet - acknowledgement of received command
    ESP_LOGD(TAG, "RSP package received");
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    std::string str(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    ESP_LOGI(TAG,  "Data string: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "HEX data %S ", pretty_data.c_str() );
    switch (data[9]) { // cmd_mnu
      case FOR_CU:
        ESP_LOGI(TAG, "Drive controller package");
        switch (data[10] + 0x80) { // sub_inf_cmd
          case RUN:
            ESP_LOGI(TAG, "RUN submenu");
			if (data[11] >= 0x80) {
			  switch (data[11] - 0x80) {  // sub_run_cmd1
			    case SBS:
			      ESP_LOGI(TAG, "Command: Step by step");
			      break;
			    case STOP:
			      ESP_LOGI(TAG, "Command: STOP");
			      break;
			    case OPEN:
			      ESP_LOGI(TAG, "Command: OPEN");
			      this->current_operation = COVER_OPERATION_OPENING;
			      break;
			    case CLOSE:
			      ESP_LOGI(TAG, "Command: CLOSE");
			      this->current_operation = COVER_OPERATION_CLOSING;
			      break;
			    case P_OPN1:
			      ESP_LOGI(TAG, "Command: Partial opening 1");
			      break;
			    case STOPPED:
			      ESP_LOGI(TAG, "Command: Stopped");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    case ENDTIME:
			      ESP_LOGI(TAG, "Operation terminated due to timeout");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    default:
			      ESP_LOGI(TAG, "Unknown team: %X", data[11]);
			  }  // switch sub_run_cmd1
			} else {
			  switch (data[11]) {  // sub_run_cmd2
			    case STA_OPENING:
			      ESP_LOGI(TAG, "Operation: Opening");
			      this->current_operation = COVER_OPERATION_OPENING;
			      break;
			    case STA_CLOSING:
			      ESP_LOGI(TAG, "Operation: Closing");
			      this->current_operation = COVER_OPERATION_CLOSING;
			      break;
			    case CLOSED:
			      ESP_LOGI(TAG, "Operation: Closed");
			      this->current_operation = COVER_OPERATION_IDLE;
			      this->position = COVER_CLOSED;
			      break;
			    case OPENED:
			      ESP_LOGI(TAG, "Operation: Open");
			      this->current_operation = COVER_OPERATION_IDLE;
			      this->position = COVER_OPEN;
			      // calibrate opened position if the motor does not report max supported position (Road 400)
                  if (this->_max_opn == 0) {
                    this->_max_opn = this->_pos_opn = this->_pos_usl;
                    ESP_LOGI(TAG, "Opened position calibrated");
                  }
			      break;
			    case STOPPED:
			      ESP_LOGI(TAG, "Operation: Stopped");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    case PART_OPENED:
			      ESP_LOGI(TAG, "Operation: Partially open");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    default:
			      ESP_LOGI(TAG, "Unknown operation: %X", data[11]);
			  }  // switch sub_run_cmd2
			}
			this->publish_state_if_changed();  // publish the status
            break; //RUN

          case STA:
            ESP_LOGI(TAG,  "Status in Motion submenu" );
            switch (data[11]) { // sub_run_cmd2
              case STA_OPENING:
              case 0x83: // Road 400
                ESP_LOGI(TAG, "Motion: Opening" );
                this->current_operation = COVER_OPERATION_OPENING;
                break;
              case STA_CLOSING:
              case 0x84: // Road 400
                ESP_LOGI(TAG,  "Motion: Closing" );
                this->current_operation = COVER_OPERATION_CLOSING;
                break;
              case CLOSED:
                ESP_LOGI(TAG,  "Motion: Closed" );
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_CLOSED;
                break;
              case OPENED:
                ESP_LOGI(TAG, "Motion: Open");
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_OPEN;
                break;
              case STOPPED:
                ESP_LOGI(TAG, "Motion: Stopped");
                this->current_operation = COVER_OPERATION_IDLE;
                request_position();
                break;
              default: // sub_run_cmd2
                ESP_LOGI(TAG,  "Motion: %X", data[11] );

                
            } // switch sub_run_cmd2

            update_position((data[12] << 8) + data[13]);
            break; //STA

          default: // sub_inf_cmd
            ESP_LOGI(TAG,  "Submenu %X", data[10] );
        }  // switch sub_inf_cmd

        break; // Drive controller package
      case CONTROL:
        ESP_LOGI(TAG,  "CONTROL package" );
        break; // CONTROL
      case FOR_ALL:
        ESP_LOGI(TAG,  "Package for all" );
        break; // FOR_ALL
      case 0x0A:
        ESP_LOGI(TAG,  "Receiver package" );
        break; // receiver package
      default: // cmd_mnu
        ESP_LOGI(TAG,  "Menu %X", data[9] );
    }  // switch  cmd_mnu


  } // else
  if ((data[6] == CMD) && (data[9] == FOR_CU)  && (data[10] == CUR_MAN) && (data[13] == NOERR)) {
    // FOR_CU responses to CMD queries that came without errors are of interest. We are looking for the status for RO600
    ///////////////////////////////////////////////////////////////////////////////////


  // RSP ответ (ReSPonce) на простой прием команды CMD, а не ее выполнение. Также докладывает о завершении операции.
  /* if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) && (data[12] == 0x19)) { // узнаём пакет статуса по содержимому в определённых байтах
     //  ESP_LOGD(TAG, "Получен пакет RSP. cmd = %#x", data[11]);
*/
    switch (data[11]) {
      case STA_OPENING:
        this->current_operation = COVER_OPERATION_OPENING;
        ESP_LOGD(TAG, "Status: Opening");
        break;
      case STA_CLOSING:
        this->current_operation = COVER_OPERATION_CLOSING;
        ESP_LOGD(TAG, "Status: Closing");
        break;
    this->publish_state();  // publish the status
   } //if
  }
  /*
    // статус после достижения концевиков
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) &&  (data[12] == 0x00)) { // узнаём пакет статуса по содержимому в определённых байтах
      ESP_LOGD(TAG, "Получен пакет концевиков. Статус = %#x", data[11]);
      switch (data[11]) {
        case OPENED:
          this->position = COVER_OPEN;
          ESP_LOGD(TAG, "Статус: Открыто");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          ESP_LOGD(TAG, "Статус: Закрыто");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Статус: Открывается");
          break;
        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Статус: Закрывается");
          break;
      } //switch
      this->publish_state();  // публикуем состояние
    } //if
  */
  // STA = 0x40,   // статус в движении
  /*
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == STA) ) { // узнаём пакет статуса по содержимому в определённых байтах
      uint16_t ipos = (data[12] << 8) + data[13];
      ESP_LOGD(TAG, "Текущий маневр: %#X Позиция: %#X %#X, ipos = %#x,", data[11], data[12], data[13], ipos);
      this->position = ipos / 2100.0f; // передаем позицию компоненту

      switch (data[11]) {
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Статус: Открывается");
          break;

        case OPENING2:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Статус: Открывается");
          break;

        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Статус: Закрывается");
          break;
        case CLOSING2:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Статус: Закрывается");
          break;
        case OPENED:
          this->position = COVER_OPEN;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Статус: Открыто");
          //      this->current_operation = COVER_OPERATION_OPENING;
          //    ESP_LOGD(TAG, "Статус: Открывается");
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Статус: Закрыто");
          //      this->current_operation = COVER_OPERATION_CLOSING;
          //ESP_LOGD(TAG, "Статус: Закрывается");
          break;
        case STOPPED:
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Статус: Остановлено");
          break;

      }  // switch

      this->publish_state();  // публикуем состояние

    } //if
  */


  ////////////////////////////////////////////////////////////////////////////////////////
} // function


void NiceBusT4::dump_config() {    //  add to the log information about the connected controller
  ESP_LOGCONFIG(TAG, "  Bus T4 Cover");
  /*ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", *this->header_[1], *this->header_[2]);*/
  switch (this->class_gate_) {
    case SLIDING:
      ESP_LOGCONFIG(TAG, "  Type: Sliding gate");
      break;
    case SECTIONAL:
      ESP_LOGCONFIG(TAG, "  Type: Sectional gate");
      break;
    case SWING:
      ESP_LOGCONFIG(TAG, "  Type: Swing gate");
      break;
    case BARRIER:
      ESP_LOGCONFIG(TAG, "  Type: Barrier");
      break;
    case UPANDOVER:
      ESP_LOGCONFIG(TAG, "  Type: Lift and swing gate");
      break;
    default:
      ESP_LOGCONFIG(TAG, "  Type: Unknown gate, 0x%02X", this->class_gate_);
  } // switch


  ESP_LOGCONFIG(TAG, "  Maximum encoder or timer position: %d", this->_max_opn);
  ESP_LOGCONFIG(TAG, "  Open goal position: %d", this->_pos_opn);
  ESP_LOGCONFIG(TAG, "  Closed gate position: %d", this->_pos_cls);

  std::string manuf_str(this->manufacturer_.begin(), this->manufacturer_.end());
  ESP_LOGCONFIG(TAG, "  Manufacturer: %S ", manuf_str.c_str());

  std::string prod_str(this->product_.begin(), this->product_.end());
  ESP_LOGCONFIG(TAG, "  Drive: %S ", prod_str.c_str());

  std::string hard_str(this->hardware_.begin(), this->hardware_.end());
  ESP_LOGCONFIG(TAG, "  Drive hardware: %S ", hard_str.c_str());

  std::string firm_str(this->firmware_.begin(), this->firmware_.end());
  ESP_LOGCONFIG(TAG, "  Drive firmware: %S ", firm_str.c_str());
  
  std::string dsc_str(this->description_.begin(), this->description_.end());
  ESP_LOGCONFIG(TAG, "  Drive description: %S ", dsc_str.c_str());


  ESP_LOGCONFIG(TAG, "  Address gateway: 0x%02X%02X", addr_from[0], addr_from[1]);
  ESP_LOGCONFIG(TAG, "  Drive address: 0x%02X%02X", addr_to[0], addr_to[1]);
  ESP_LOGCONFIG(TAG, "  Receiver address: 0x%02X%02X", addr_oxi[0], addr_oxi[1]);
  
  std::string oxi_prod_str(this->oxi_product.begin(), this->oxi_product.end());
  ESP_LOGCONFIG(TAG, "  Receiver: %S ", oxi_prod_str.c_str());
  
  std::string oxi_hard_str(this->oxi_hardware.begin(), this->oxi_hardware.end());
  ESP_LOGCONFIG(TAG, "  Receiver hardware: %S ", oxi_hard_str.c_str());

  std::string oxi_firm_str(this->oxi_firmware.begin(), this->oxi_firmware.end());
  ESP_LOGCONFIG(TAG, "  Receiver firmware: %S ", oxi_firm_str.c_str());
  
  std::string oxi_dsc_str(this->oxi_description.begin(), this->oxi_description.end());
  ESP_LOGCONFIG(TAG, "  Receiver description: %S ", oxi_dsc_str.c_str());
 
  ESP_LOGCONFIG(TAG, "  Auto-closing - L1: %S ", autocls_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Close up after the photo - L2: %S ", photocls_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Always close - L3: %S ", alwayscls_flag ? "Yes" : "No");
  
}




//management team building
std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {this->addr_to[0], this->addr_to[1], this->addr_from[0], this->addr_from[1]}; // header
  frame.push_back(CMD);  // 0x01
  frame.push_back(0x05);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(CONTROL);
  frame.push_back(RUN);
  frame.push_back(control_cmd);
  frame.push_back(0x64); // OFFSET CMD, DPRO924 refused to work with 0x00, although other drives responded to commands
  uint8_t crc2 = (frame[7] ^ frame[8] ^ frame[9] ^ frame[10]);
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // to output the command to the log
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "A team has been formed: %S ", pretty_cmd.c_str() );

  return frame;
}

// INF command generation with and without data
std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len) {
  std::vector<uint8_t> frame = {to_addr1, to_addr2, this->addr_from[0], this->addr_from[1]}; // header
  frame.push_back(INF);  // 0x08 mes_type
  frame.push_back(0x06 + len); // mes_size
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(whose);
  frame.push_back(inf_cmd);
  frame.push_back(run_cmd);
  frame.push_back(next_data); // next_data
  frame.push_back(len);
  if (len > 0) {
    frame.insert(frame.end(), data.begin(), data.end()); // data block
  }
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < 12 + len; i++) {
    crc2 = crc2 ^ frame[i];
  }
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // to output the command to the log
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "INF package generated: %S ", pretty_cmd.c_str() );

  return frame;

}


void NiceBusT4::send_raw_cmd(std::string data) {

  std::vector < uint8_t > v_cmd = raw_cmd_prepare (data);
  send_array_cmd (&v_cmd[0], v_cmd.size());

}


//  Here we need to add a check for incorrect data from the user
std::vector<uint8_t> NiceBusT4::raw_cmd_prepare (std::string data) { // preparation of user-entered data to be able to send
// delete everything except hexadecimal letters and numbers.
data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch) {
    return (!(isxdigit(ch)) );
  }), data.end()); 

  //assert (data.size () % 2 == 0); // parity check
  std::vector < uint8_t > frame;
  frame.resize(0); // resize the command

  for (uint8_t i = 0; i < data.size (); i += 2 ) { // fill the command array
    std::string sub_str(data, i, 2); // take 2 bytes from the command
    char hexstoi = (char)std::strtol(&sub_str[0], 0 , 16); // convert to a number
    frame.push_back(hexstoi);  // write the number to the line item of the new command
  }


  return frame;

}



void NiceBusT4::send_array_cmd (std::vector<uint8_t> data) {          // sends break + the command prepared earlier in the array
  return send_array_cmd((const uint8_t *)data.data(), data.size());
}
void NiceBusT4::send_array_cmd (const uint8_t *data, size_t len) {
  // sending data to uart
  uint8_t br_ch = 0x00;                                               // to break
  this->parent_->flush();                                               // clear uart
  this->parent_->set_baud_rate(BAUD_BREAK);                            // undercutting
  this->parent_->load_settings(false);
  this->parent_->write_byte(br_ch);                                    // sending zero at low speed, long zero.
  this->parent_->flush();
  delayMicroseconds(90);                                          // add a delay to the wait, otherwise the speed will switch before sending. With the delay on d1-mini I got a perfect signal, break = 520us

  this->parent_->set_baud_rate(BAUD_WORK);                             // getting back to work awake
  this->parent_->load_settings(false);
  this->parent_->write_array(data, len);                                // sending the main parcel
  this->parent_->flush();

  std::string pretty_cmd = format_hex_pretty((uint8_t*)&data[0], len);                    // to output the command to the log
  ESP_LOGI(TAG,  "Posted: %S", pretty_cmd.c_str());

}


// generating and sending inf commands from yaml configuration
void NiceBusT4::send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command, std::string next_data, bool data_on, std::string data_command) {
  std::vector < uint8_t > v_to_addr = raw_cmd_prepare (to_addr);
  std::vector < uint8_t > v_whose = raw_cmd_prepare (whose);
  std::vector < uint8_t > v_command = NiceBusT4::raw_cmd_prepare (command);
  std::vector < uint8_t > v_type_command = raw_cmd_prepare (type_command);
  std::vector < uint8_t > v_next_data = raw_cmd_prepare (next_data);
  std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);


  if (data_on) {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0], v_data_command, v_data_command.size()));
  } else {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0]));
  } // else
}

// generating and sending setup commands to the drive controller from a yaml configuration with minimal parameters
void NiceBusT4::set_mcu(std::string command, std::string data_command) {
    std::vector < uint8_t > v_command = raw_cmd_prepare (command);
    std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);
    tx_buffer_.push(gen_inf_cmd(0x04, v_command[0], 0xa9, 0x00, v_data_command));
  }
  
// device initialization
void NiceBusT4::init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device ) {
  if (device == FOR_CU) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00)); // drive type request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00)); // manufacturer request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  firmware request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //product request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //hardware request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));   //opening position request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00)); // closing position request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //description request
    if (is_walky)  // requesting the maximum value for the encoder
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00, {0x01}, 1));
    else
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));
    request_position();  // current position request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00)); //Gate status (Open/Closed/Stopped)
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, AUTOCLS, GET, 0x00)); // Auto-closing
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, PH_CLS_ON, GET, 0x00)); // Close after Photo
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, ALW_CLS_ON, GET, 0x00)); // Always close
  }
  if (device == FOR_OXI) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //product request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //hardware request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  firmware request    
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //description request    
  }
  
}

// Requesting the conditional current position of the actuator
void NiceBusT4::request_position(void) {
  if (is_walky)
    tx_buffer_.push(gen_inf_cmd(this->addr_to[0], this->addr_to[1], FOR_CU, CUR_POS, GET, 0x00, {0x01}, 1));
  else
    tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));
}

// Updating the current position of the actuator
void NiceBusT4::update_position(uint16_t newpos) {
  last_position_time = millis();
  _pos_usl = newpos;
  position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
  ESP_LOGI(TAG, "Conditional goal position: %d, position in %%: %.3f", newpos, position);
  if (position < CLOSED_POSITION_THRESHOLD) position = COVER_CLOSED;
  publish_state_if_changed();  // publish the status
  
  if ((position_hook_type == STOP_UP && _pos_usl >= position_hook_value) || (position_hook_type == STOP_DOWN && _pos_usl <= position_hook_value)) {
  	ESP_LOGI(TAG, "The desired position has been reached. Stopping the gate");
  	send_cmd(STOP);
  	position_hook_type = IGNORE;
  }
}

// Publishing the gate status when the gate changes
void NiceBusT4::publish_state_if_changed(void) {
  if (current_operation == COVER_OPERATION_IDLE) position_hook_type = IGNORE;
  if (last_published_op != current_operation || last_published_pos != position) {
    publish_state();
    last_published_op = current_operation;
    last_published_pos = position;
  }
}

}  // namespace bus_t4
}  // namespace esphome