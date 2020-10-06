@ECHO OFF

cd ..
cd ..
cd C:\DiscVisionDeluxe\dvd_DgrafX\Source\DiscVisionDeluxeUE
g++ DfisX/main.cpp DfisX/dfisx.cpp DfisX/daero.cpp DfisX/dgyro.cpp DfisX/dpropagate.cpp DfisX/dio.cpp -o DfisX/main
cd DfisX
main hyzer -1 pitch 0.08 discmold 6 posx -0.619 posy 0.344 posz 1.941 velx 30 vely 0.389 velz 6.5 spinrate -130 wobble 0 windx 0 windy 0 windz 0
pause
