def UlinApprox(u):

    res = (u+6.3810)/10.354464663188761
    print("U approx: ", res)
    return res


def SbLinApprox(s):

    Sb = 1/s
    res = (Sb - 0.0002) / 0.0025
    print("Dist approx: ", res)
    return res


blobs = 64.2599

xval = 157.2062

x = UlinApprox(xval)
blob = SbLinApprox(blobs)

print(x)
print(blob)
