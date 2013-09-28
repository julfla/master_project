#!/bin/sh

echo "This test will download, convert, compute descriptors and distance map"

#TODO
#add the plot scripts and test them
#add to git repo
#add scripts to cmakes and redefine targets
# test mesh randomness

dataset=~/Documents/Recherche/dataset
tool_dir=$(dirname $(readlink -f $0))/tools
bin_dir=$(dirname $(readlink -f $0))/bin
#tmp_dir=$(mktemp -d)
tmp_dir=~/Bureau/tmp; mkdir -p $tmp_dir

cd $tmp_dir
#rm */*.dist;


mkdir -p mug; cd mug
echo "Downloading skp for keywords \"mug\""
ruby $tool_dir/warehouse_scrapper.rb mug
cd ..

#echo "Link to the dataset files"
#for dir in $dataset/*/; do
#	mkdir -p $(basename $dir);
#	for file in $dir/*.skp; do ln -sf $file $(basename $dir)/$(basename $file); done;
#done

echo "Convert skp files to tri format"
#for file in */*.skp; do WINEDEBUG=-all, wine $bin_dir/skp2tri.exe -i $file -o ${file%.skp}.tri; done;

echo "Computation of descriptors"
#for file in */*.tri; do $bin_dir/distribution -i $file -o ${file%.tri}.dist; done;

echo "Plot descriptors"
for file in */*.dist; do
	$bin_dir/distribution --csv --conv -i $file -o ${file%.dist}.png
	R --slave -f $tool_dir/plot_raw_dist.R --args $file ${file%.dist}.png
done;

echo "Computation of distance matrix"
files=*/*.dist
matrix="model"
for file in $files; do matrix=$matrix" "$(basename --suffix=".dist" -a $file); done;
matrix=$matrix"\n"
for file1 in $files; do
	matrix=$matrix$(basename --suffix=".dist" -a $file1)
#	for file2 in $files; do matrix=$matrix" "$($bin_dir/distance $file1 $file2); done;
	matrix=$matrix"\n"
done
echo $matrix > matrix_distance.m

R --slave -f $tool_dir/heatmap.R --args "matrix_distance.m" 

#rm -r $tmp_dir


