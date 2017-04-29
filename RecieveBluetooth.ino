/*
   These functions deal with recieving inforamtion
   about the storage and supply rod availability.
   This availability should be checked after the robot alters
   the position of any rod on the field (insertions & extractions).
*/

//TODO
//reads any available messages on the bluetooth serial input
void readMessages() {
  // attempt to read a message (packet)
  // the only messages returned are those that are broadcast or sent specifically to this robot
  if (btmaster.readPacket(pkt)) {                  // if we have received a message
    if (pcol.getData(pkt, data, type)) {           // see if we can extract the type and data
      if (pkt[4] == 0x00 || pkt[4] == 0x06) {    //see if this message is from the field computer & adressed to this robot or all robots, ignore it if not data[3] == 0x00 && 
        switch (type) {                            // process the message based on the type
          case 0x01:                               // received a storage tube message
            storageData = data[0];                 // extract and save the storage-related data (the byte bitmask)
            break;
          case 0x02:                               // received a supply tube message
            supplyData = data[0];                  // extract and save the supply-related data (the byte bitmask)
            break;
          case 0x04:                               //recieved Stop Movement Message
            stopDrive();                           //stop current robot translations
            driveFlag = false;                     //stop future robot tranlatetion
            break;
          case 0x05:                               //recieved resume movement message
            driveFlag = true;                      //re-start movement FROM WHERE THE ROBOT LEFT OFF
            break;
          default:                                 // ignore other types of messages
            break;
        }
      }
    }
  }
}

