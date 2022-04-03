import numpy

# Compare differences when doing dict search
# Run For Loop

# Run Broadcasting (might be faster?)
# Broadcasting provides a means of vectorizing array operations so that looping occurs in C instead of Python. It does this without making needless copies of data and usually leads to efficient algorithm implementations.


def link_0 () :
    return 'link_0_dict'

def link_1 () :
    return 'link_1_dict'


if __name__ == "__main__":
    switcher = {0:link_0, 1:link_1}
    print(switcher[0]())