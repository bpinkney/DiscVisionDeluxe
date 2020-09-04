@ECHO OFF

cd ..
cd ..
cd C:\Users\Crom\Documents\GitHub\DiscVisionDeluxe\dvd_DfisX
g++ main.cpp dfisx.cpp daero.cpp dgyro.cpp dpropagate.cpp dio.cpp -o main
main hyzer -0.55 pitch 0.15 discmold 0 posx -0.619 posy 0.344 posz 1.941 velx 27 vely 0.389 velz 6 spinrate -120 wobble 0 matlab 1 windx 0 windy 0 windz 0
pause
