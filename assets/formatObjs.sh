if [ "$1" = "--help" ]; then
	echo "./formatObjs.sh [folder containing .obj files] [scale]"
else
	scale=$2
	if [ -z "$scale" ]; then
		scale=1;
	fi

	mkdir $1/copy
	for file in $(ls -p $1 | grep -v / | grep "\.obj$")
	do
	    cat $1/$file | awk -v scale="$scale" '{if ($1 == "v") {print $1 " " $2*scale " " $3*scale " " $4*scale} else if ($1 == "f") {print $0}}' | sort -srk 1,1 > "$1/copy/$file"
	done
	mv $1/copy/*.obj $1/
	rm -rf $1/copy
fi