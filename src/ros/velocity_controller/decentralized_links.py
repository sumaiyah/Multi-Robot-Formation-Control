#The robot links for decentralization, as indexes into the formation

LINE_LINKS = {
    0: [1],
    1: [0, 2],
    2: [1, 3],
    3: [2, 4],
    4: [3]
}

COLUMN_LINKS = {
    0: [1],
    1: [0, 2],
    2: [1, 3],
    3: [2, 4],
    4: [3]
}

DIAMOND_LINKS = {
    0: [2],
    1: [2],
    2: [0, 1, 3, 4],
    3: [2],
    4: [2]
}

WEDGE_LINKS = {
    0: [1],
    1: [0, 2],
    2: [1, 3],
    3: [2, 4],
    4: [3]
}
