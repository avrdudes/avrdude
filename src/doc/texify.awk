#!/usr/bin/awk

$2 ~ /^=$/ {
	desc = $0
	code = $1
	sub("[^=]+=[[:space:]]*","", desc)
	printf("@item @code{%s} @tab %s\n", code, desc)
}
