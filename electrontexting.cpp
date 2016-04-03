void sendSMS_manual(String message, String phoneNumber){
//probably want delays for this
  String setPhone = "AT+CMGS=";
  setPhone+=phoneNumber;
  int numberSet = Cellular.command(setPhone);
  int messageSent = Cellular.command(message);
  int messageEnded = Cellular.command(String((char)26).c_str()); //the ASCII code of the ctrl+z is 26)
}

