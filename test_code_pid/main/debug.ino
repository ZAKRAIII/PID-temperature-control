void debug(){
  Serial.print("suhu = " + String(suhu) + " \t");

  Serial.println("PID_error = " + String(PID_error) + " \t");
  Serial.print("PID_p = " + String(PID_p) + " \t");
  Serial.print("PID_i = " + String(PID_i) + " \t");
  Serial.print("PID_d = " + String(PID_d) + " \t");

  Serial.print("PID_value = " + String(PID_value) + " \t");
  Serial.print("signal_value = " + String(signal_value) + " \t");
}
