def main(): 
    # Write to file
    with open("servo.waypoints", "w+") as ofile:
        ofile.write('QGC WPL 110\n')
        ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (0,0,0,183,9,1100,0,0,0,0,0,1))
        ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (1,0,0,112,3,0,0,0,0,0,0,1))
        ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (2,0,0,183,9,1900,0,0,0,0,0,1))
        

if __name__=="__main__":
    main()