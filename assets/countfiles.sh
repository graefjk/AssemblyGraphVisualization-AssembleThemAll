cd ./multi_assembly
for folder in $(ls)
do
    echo $folder $(ls $folder/*.obj -1 | wc -l) >> ../filecount.txt
done