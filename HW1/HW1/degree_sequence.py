import util
def Havel_Hakimi(sequence=list):
    if len(sequence) ==0:
        print("length of degree sequence is 0")
    sequence.sort(reverse=True)
    flag = False
    print(sequence)
    while (len(sequence) != 0):
        num = sequence.pop(0)
        print(sequence)
        if len(sequence) == 0:
            flag = True
            print("The graph does not always exist with this degree sequence.")
        for i in range(num):
            sequence[i] -=1
            if sequence[i] < 0:
                flag = True
                print("The graph does not always exist with this degree sequence.")
                break
        print(sequence)
        if flag: break
        sequence.sort(reverse=True)
        print(sequence)
        if sum(sequence) == 0:
            print("There exists a graph with the given degree sequence.")
            break
        

def generate_sequence():
    randomlist = []
    for i in range(0,10):
        n = util.random.randint(1,10)
        randomlist.append(n)
    return randomlist
def main():
    # sequence = generate_sequence()
    
    sequence1 = [3, 3, 3, 3, 5, 6, 6, 6, 6, 6, 6]
    # Havel_Hakimi(sequence=sequence1)
    print('--------------------------------------------------')
    # sequence2 = [1, 1, 3, 3, 3, 3, 5, 6, 8, 9]
    sequence2 = [6 ,3, 3, 3, 3, 2, 2, 2, 2, 1, 1]

    Havel_Hakimi(sequence=sequence2)
if __name__=="__main__": 
    main() 