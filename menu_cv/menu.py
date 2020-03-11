import png

# Load an color image in grayscale





def invert_2dmatrix(data):
    for i in range(len(data)):
            for j in range(len(data[0])):
                data[i][j] = 255 - data[i][j]
    
    return data


def split_menu(filename):
    return 0

"""
PNG images are structured in a 2D array of integers/bytes.
If my data array is data[x][y], our axes are as follows:


  * ------> y+
  |(0,0)
  |
  |
  v
  x+

Provide the top left, bottom right coordinate pairs to grab a piece of the image.
"""

# Dictionary of png coordinate values for identifying meals within a week-long menu.
# Format is 'day' : coords[tl_x, tl_r, br_x, br_y]
COORD_DICT = {'week' : [146, 129, 560, 758], 'monday' : [146, 129, 560, 250], 'tuesday' : [146, 251, 560, 376], 'wednesday' : [146, 377, 560, 500], 'thursday' : [146, 501, 560, 627], 'friday' : [146, 628, 560, 758]}
r = png.Reader("menu.png")
COLOR_PALETTE = r.read()[3]['palette']

def write_meal(menuData, weekday):
    if not weekday in COORD_DICT.keys():
        print("Invalid weekday")
        return 0
    else:
        coords = COORD_DICT[weekday]
        print(coords)
        subimg = grab_subsquare_tl_br(menuData, coords[0], coords[1], coords[2], coords[3])
        write_subimage(weekday + ".png", subimg, COLOR_PALETTE)

def grab_subsquare_tl_br(data, tl_x, tl_y, br_x, br_y):
    assert(len(data) > 0)

    croppedData = []
    currx = tl_x
    curry = tl_y

    while currx < br_x:
        row = []
        while curry < br_y:
            row.append(data[currx][curry])
            curry += 1
        currx += 1
        curry = tl_y
        croppedData.append(row)

    return croppedData
    
def write_subimage(filename, data, color_palette):
    f = open(filename, 'wb')
    w = png.Writer(len(data[0]), len(data), palette=color_palette)
    w.write(f, data)
    f.close()

def png_to_int_arr(filename):
    r = png.Reader(filename)
    rowIterator = r.read()[2]
    data = []
    for row in rowIterator:
        data.append([row[i] for i in range(len(row))])

    return data



if __name__ == "__main__":
    data = png_to_int_arr('menu.png')
    for date in COORD_DICT.keys():
        write_meal(data, date)
    
