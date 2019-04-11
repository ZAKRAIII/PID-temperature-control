ISR(PCINT0_vect){
  //Push button was pressed!
  if (PINB & B00001000){
    buttUpFlag = true;
  }

  if(PINB & B00000100){
    buttDownFlag = true;
  }

}
