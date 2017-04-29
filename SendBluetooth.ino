/*
 * These functions send various types of messages to
 * the field controller. They take their structure from the 
 * example messages in the sample code
 */
// generate and send the heartbeat message
void sendHBMessage() {
  pcol.setDst(0x00);                                   // this will be a broadcast message
  sz = pcol.createPkt(0x07, data, pkt);                // create a packet using the heartbeat type ID (there is no data)
  btmaster.sendPkt(pkt, sz);                           // send packet, value pointed to by sz to the field computer
}

//sends radiation alert message (new fuel rod)
void sendNewRodAlert() {
  pcol.setDst(0x00);                     // this will be a broadcast message
  data[0] = 0xFF;                        // indicate a new fuel rod
  sz = pcol.createPkt(0x03, data, pkt);  // create a packet using the radiation alert type ID (1 byte of data used this time)
  btmaster.sendPkt(pkt, sz);             // send to the field computer
}

//sends radiation alert message (spent fuel rod)
void sendSpentRodAlert() {
  pcol.setDst(0x00);                     // this will be a broadcast message
  data[0] = 0x2C;                        // indicate a spent fuel rod
  sz = pcol.createPkt(0x03, data, pkt);  // create a packet using the radiation alert type ID (1 byte of data used this time)
  btmaster.sendPkt(pkt, sz);             // send to the field computer
}

//sends message with the given status of the robot (whether the robot is moving, is it holding something, is the arm moving?)
void sendRobotStatus(byte movement, byte gripper, byte operation) {
  pcol.setDst(0x00);                     // this will be a broadcast message
  data[0] = movement;                    // indicate a movement status
  data[1] = gripper;                     // indicate gripper status
  data[2] = operation;                   // indicate operation status
  sz = pcol.createPkt(0x06, data, pkt);  // create a packet using the robot status type ID (3 bytes of data)
  btmaster.sendPkt(pkt, sz);             // send to the field computer
}
