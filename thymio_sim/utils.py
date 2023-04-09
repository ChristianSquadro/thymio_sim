from collections import Counter

def list_mode(lst):
    data = Counter(lst)
    return data.most_common(1)[0][0]