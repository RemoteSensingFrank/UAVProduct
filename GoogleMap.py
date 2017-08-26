import urllib
import urllib2

def GetImageForGoogleUrl(url,save):
    headers=("User-Agent","Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:53.0) Gecko/20100101 Firefox/53.0")
    opener = urllib2.build_opener()
    opener.addheaders = [headers]
    urllib.urlretrieve(url,save)

