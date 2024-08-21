# Transform the avrdude -p \? output into a texinfo table
s/^ *\([^ ]*\) *= \([^(]*\).(/@cindex @code{\1}\n@cindex \2\n@item @code{\1} @tab \2 (/
