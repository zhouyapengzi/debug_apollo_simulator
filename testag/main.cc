#include <iostream>
#include "add.cc"
void hello(){
  std::cout << "Hello" << std::endl;
}

int main(){
  hello(); //print "Hello"
  add();
  return 0;
}
