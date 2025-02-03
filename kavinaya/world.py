#code something hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
#kolattam
import random
print('Kolaattam')
ys=input('Aaduvoma? = Aan or No:')
while ys=='Aan':
    a=random.randint(1,6)
    b=random.randint(1,6)
    s=a+b
    if (s==7 or s==11):
        print('Bhaiiii Mass kaatureenga!!!',s)
        ys=input('Aaduvoma? = Aan or No:')
    elif (s==2 or s==3 or s==12):
        print('Ada Ponga Bhai,you got :(((',s)
        ys=input('Aaduvoma? = Aan or No:')
    else:
        print('Idaan kedachudhu',s)
        ys=input('Aaduvoma? = Aan or No:')
        while ys=='Aan':
            a=random.randint(1,6)
            b=random.randint(1,6)
            t=a+b
            if t==7:
                print('Ada Ponga Bhai,you got :(((',t)
                ys=input('Aaduvoma? = Aan or No:')
                break
            if t==s:
                print('Bhaiiii Mass kaatureenga!!!',t)
                ys=input('Aaduvoma? = Aan or No:')
                break
            else:
                print('Idaan kedachudhu',t)
                ys=input('Aaduvoma? = Aan or No:')
                continue
        else:
            break
print('Nandri Vanakkam!')
                        
