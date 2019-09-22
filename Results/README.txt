check main.cpp
	make sure it has the correct ammount of generations
	make sure it has the appropriate amount of cores is set
		population size = cores
cd into build and run: "make"
cd into Results
	make directory for the seed value (i.e. 12345)
cd into seed directory
run: "../../build/embedding/mpga_emergent_behavior 12345"
run: rm -f AR*
run: rm -f sc(tab)*
cd Results
./analyze-all-linux -dir 12345/ -workers 100 -out master_scores_12345.csv
	Make sure that -dir is the correct seed directory
	Make sure that the -out file has a unique name