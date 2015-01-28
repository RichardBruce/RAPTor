FROM_FILE="act"

# Pick golden files
if [ ! -z "$2" ]; then
    FROM_FILE="$2"
fi

for file in $1/*$FROM_FILE* ; do mv $file ${file/$FROM_FILE/exp}; done