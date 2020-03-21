function ld = dvd_DvisEst_load_csv_log(filepath)

    M = csvread(filepath, 1, 0);

    ld.time_s      = M(:, 1);
    pos_x       = M(:, 2);
    pos_y       = M(:, 3);
    pos_z       = M(:, 4);
    pos_x_ground = mean(pos_x)
    pos_yground = mean(pos_y)
    pos_z_ground = mean(pos_z)
    ld.Qwxyz(:, 1) = M(:, 5);
    ld.Qwxyz(:, 2) = M(:, 6);
    ld.Qwxyz(:, 3) = M(:, 7);
    ld.Qwxyz(:, 4) = M(:, 8);
    ld.R00         = M(:, 9);
    ld.R01         = M(:, 10);
    ld.R02         = M(:, 11);
    ld.R10         = M(:, 12);
    ld.R11         = M(:, 13);
    ld.R12         = M(:, 14);
    ld.R20         = M(:, 15);
    ld.R21         = M(:, 16);
    ld.R22         = M(:, 17);

    ld.pos_xyz = [pos_x, pos_y, pos_z];

    disp(sprintf('%d entries over %0.4f seconds', length(ld.time_s), max(ld.time_s) - min(ld.time_s)));

end