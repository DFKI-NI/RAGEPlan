#!/bin/sh

### Problem definition
problem=cellar #Problem name: rocksample, cellar, etc.
inputFile=../Problems/cellar.prob

### Algorithm parameters
rolloutKnowledge=3 # 1 = random, 2 = preferred actions, 3 = PGS
fTable=1 #IRE y/n

### Search and experiment parameters
minDoubles=16
maxDoubles=16
numSteps=100
runs=1

### Output
outputFile=output.txt
verbose=1

./rage --problem $problem --inputFile $inputFile --minDoubles $minDoubles --maxDoubles $maxDoubles --numSteps $numSteps --runs $runs --rolloutKnowledge $rolloutKnowledge --fTable $fTable --verbose $verbose --outputFile $outputFile
