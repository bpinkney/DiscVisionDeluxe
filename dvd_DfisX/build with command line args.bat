@ECHO OFF

cd ..
cd ..
cd src
g++ main.cpp dfisx.cpp daero.cpp dgyro.cpp dpropagate.cpp -o main
main hyzer 0.224 pitch -0.192 discmold 0 posx 0 posy 0 posz 2 velx 11 vely 3.6 velz 1.66 spinrate -46 wobble 0 
pause
