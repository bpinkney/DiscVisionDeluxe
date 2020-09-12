@ECHO OFF

cd ..
cd ..
cd C:\DiscVisionDeluxe\dvd_DfisX
g++ main.cpp dfisx.cpp daero.cpp dgyro.cpp dpropagate.cpp dio.cpp -o main
main hyzer -1 pitch 0.08 discmold 0 posx -0.619 posy 0.344 posz 1.941 velx 30 vely 0.389 velz 6.5 spinrate -130 wobble 0 matlab 1 windx 0 windy 0 windz 0
pause
