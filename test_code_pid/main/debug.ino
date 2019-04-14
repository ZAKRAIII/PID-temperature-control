void debug(){
  Serial.print("suhu = " + String(suhu) + " \t");
  Serial.print("PID_p = " + String(PID_p) + " \t");
  //Serial.println(PID_p);
  Serial.print("PID_p = " + String(PID_p) + " \t");
  //Serial.println(PID_i);
  Serial.print("PID_i = " + String(PID_i) + " \t");
  //Serial.println(PID_d);
  Serial.print("PID_d = " + String(PID_d) + " \t");

  //Serial.print(PID_value);
  Serial.print("PID_value = " + String(PID_value) + " \t");
  //Serial.println(signal_value);
  Serial.print("signal_value = " + String(signal_value) + " \t");

  //Serial.print(PID_error);
  Serial.println("PID_error = " + String(PID_error) + " \t");
}
