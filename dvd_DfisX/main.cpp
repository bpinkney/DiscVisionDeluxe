#include "DfisX.hpp"
#include "Dio.hpp"
#include <iostream> 
#include <string> 
#include <sstream>


int main(int argc, char *argv[]) 
{

  //send command line arguments to the parser in Dio
  int argcount = argc;
  char** argvalues = argv;
  if (argcount>1 ) parse_cl (argcount,  argvalues);
  else std::cout << "\n No command line parameters passed";
  
}