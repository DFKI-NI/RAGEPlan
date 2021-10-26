#!/bin/sh
while 1; do
   mem=`ps aux | grep ./rage | awk 'NR==1 {print $4}'`;
   if [ $mem -gt $max ]; then
      max=$mem;
   fi
   echo -e "Max mem: "$mem;
   sleep 1;
done
