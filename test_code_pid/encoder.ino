ISR(PCINT0_vect){
  //Push button was pressed!
  if (PINB & B00001000){
//    set_temperature++;
    butt_naik = true;
  }

  if(PINB & B00000100){
//    set_temperature--;
    butt_turun = true;
  }

}
