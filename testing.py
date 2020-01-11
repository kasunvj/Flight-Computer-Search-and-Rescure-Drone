a = ["a0","b0","c","t0"]

for file in a:
	print(file)

print("--------------")

for file in a:
	if "0" in file:
		print(file)

# a0
# b0
# c
# t0
# --------------
# a0
# b0
# t0