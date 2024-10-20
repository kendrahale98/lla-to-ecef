// A simple program to sum numbers and report it to the console.

#include<iostream>

#include "simple.h"

// Main function, sums integers and prints outputs.
int main() {
  std::cout << "Welcome to the simple summing program." << std::endl;
  int ints[] = {0 , 1, 2, 3, 4, 5, 6, 7, 8, 9};

  for (int i = 0; i < ((sizeof(ints) / sizeof(ints[0])) - 1); i++) {
    int a = ints[i];
    int b = ints[i+1]; 
    std::cout << "The sum of " << a << " and " << b << " is " << Sum(a, b) << "." << std::endl;
  }
}