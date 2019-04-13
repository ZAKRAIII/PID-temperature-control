ISR(PCINT1_vect){
  if(PINC & 0b00000001){
    buttSelFlag = true;
  }
  if(PINC & 0b00000010){
    buttUpFlag = true;
  }
  if(PINC & 0b00000100){
    buttDownFlag = true;
  }
}
