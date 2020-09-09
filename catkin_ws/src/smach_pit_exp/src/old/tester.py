import csv

wp_2_pit = []

with open('waypoints.csv', 'rb') as f:
	reader = csv.reader(f, delimiter=',')
	header = next(reader)
	for row in reader:
		tmp = []
		for elem in row:
			tmp.append(int(elem))
		wp_2_pit.append(tmp)
	print wp_2_pit.size()
	