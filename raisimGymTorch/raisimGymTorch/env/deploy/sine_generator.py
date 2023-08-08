from math import sin,pi

def sine_generator(angle_list, idx, T, rate=1):
    base1= 0.82
    base3= 0.0
    base2 = -2 * base1
    ang = abs(sin( float(idx) / T  * pi)) * rate

    idx_base = 0

    if (int(idx/T) % 2 ) == 1:
        angle_list[idx_base+1] = ang + base1
        angle_list[idx_base+2] = -2 * ang + base2

        angle_list[idx_base+4] = base1
        angle_list[idx_base+5] = base2

        angle_list[idx_base+7] = base1
        angle_list[idx_base+8] = base2
        angle_list[idx_base+10] = ang + base1
        angle_list[idx_base+11] = - 2 * ang + base2
    else:
        angle_list[idx_base+1] = base1
        angle_list[idx_base+2] = base2

        angle_list[idx_base+10] = base1
        angle_list[idx_base+11] = base2
        angle_list[idx_base+4] = ang + base1
        angle_list[idx_base+5] = -2 * ang + base2

        angle_list[idx_base+7] = ang + base1
        angle_list[idx_base+8] = -2 * ang + base2
    angle_list[idx_base+0] = base3
    angle_list[idx_base+3] = base3
    angle_list[idx_base+6] = base3
    angle_list[idx_base+9] = base3
    return angle_list




if __name__=='__main__':
    angle_list = [0 for x in range(12)]
    sine_generator(angle_list, 2, 40, 0.3)
    print(angle_list)