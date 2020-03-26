@ECHO OFF

cd ..
cd ..
cd src
g++ main.cpp dfisx.cpp daero.cpp dgyro.cpp dpropagate.cpp -o main
main test 1
pause
