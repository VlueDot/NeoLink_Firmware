


String sn_tool_version = "0.0.1";
String DEVICE;
//______________________________________________________________________
//
// Codigo principal
//______________________________________________________________________


void setup() { 

  Serial.begin(115200);
  delay(4000);
  

}

void loop() {

  Serial.println("SN_Tool " + sn_tool_version);
  Serial.println("________________________________");
  Serial.println("Select an option: \n\t1: SN assignment\n\t2: Assign existing SN");
  Serial.println("________________________________");
  Serial.print("Waiting input.. ");
  while(!Serial.available());
  String option = Serial.readString();
  Serial.println(option);
  if (option == "1\n"){
      Serial.println("Select an device: \n\t1: NL\n\t2: NN\n\tq: quit");
      while(!Serial.available());
      option = Serial.readString();
      Serial.println(option);


  }

  else if (option == "2") {


  }

  else Serial.println ("Unknown entry. Try again.");



}
