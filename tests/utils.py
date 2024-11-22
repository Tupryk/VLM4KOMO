import robotic as ry


def draw_uniform_block(C, range_x, range_y, range_z):
    C.addFrame('block'). setShape(ry.ST.box, size=[range_x[1]-range_x[0], range_y[1]-range_y[0], range_z[1]-range_z[0]]) .setPosition([(range_x[1]+range_x[0])/2, (range_y[1]+range_y[0])/2, (range_z[1]+range_z[0])/2]) .setColor([1,0,0,.3])
    #C.addFrame('block_center'). setShape(ry.ST.marker, [.3]) .setPosition([(range_x[1]+range_x[0])/2, (range_y[1]+range_y[0])/2, (range_z[1]+range_z[0])/2]) .setColor([1,0,0,.5])
    #print((range_x[1]+range_x[0])/2, (range_y[1]+range_y[0])/2, (range_z[1]+range_z[0])/2)