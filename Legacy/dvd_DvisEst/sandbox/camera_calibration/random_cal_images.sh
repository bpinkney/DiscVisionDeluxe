dir1="cal_imgs_raw/"
dir2="cal_imgs/"

rm -r "$dir2"
mkdir "$dir2"

ls cal_imgs_raw/|sort -R |tail -50 |while read file; do
    # Something involving $file, or you can leave
    # off the while to just get the filenames
    cp "$dir1$file" "$dir2$file"
done