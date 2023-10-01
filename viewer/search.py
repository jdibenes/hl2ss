import os
import requests
import json

dictionary_api = 'https://api.dictionaryapi.dev/api/v2/entries/en/'
google_search_url = 'https://www.google.com/search?q='
# sample request

#[{"word":"ok","phonetics":[],"meanings":[{"partOfSpeech":"adjective","definitions":[{"definition":"All right, permitted.","synonyms":[],"antonyms":[],"example":"Do you think it's OK to stay here for the night?"},{"definition":"Satisfactory, reasonably good; not exceptional.","synonyms":[],"antonyms":[],"example":"The soup was OK, but the dessert was excellent."},{"definition":"In good health or a good emotional state.","synonyms":[],"antonyms":[],"example":"He's not feeling well now, but he should be OK after some rest."}],"synonyms":["all right","allowed","permissible","fine","well","adequate","all right","not bad","satisfactory"],"antonyms":["forbidden","ill","poorly","sick","under the weather","unwell","bad","inadequate","poor","unsatisfactory"]}],"license":{"name":"CC BY-SA 3.0","url":"https://creativecommons.org/licenses/by-sa/3.0"},"sourceUrls":["https://en.wiktionary.org/wiki/OK","https://en.wiktionary.org/wiki/ok"]}]

def dictionary(word):
    x = requests.get(dictionary_api+word)
    print(x.text)
    result = json.loads(x.text)
    if len(result) > 0:
        word = result[0]['word']
    # TODO: implement word type and other stuff
    # TODO: add decoration
        description = ''
        meanings = result[0]['meanings']
        for meaning in meanings:
            description += '<i>' + meaning['partOfSpeech'] + '</i>'+ '\n'
            definitions = meaning['definitions']
            for definition in definitions:
                if 'definition' in definition: description += definition['definition'] + '\n'
                if 'synonyms' in definition: description += 'Synonyms: ' + str(definition['synonyms'])[1:-1] + '\n'
                if 'antonyms' in definition: description += 'Antonyms: ' + str(definition['antonyms'])[1:-1] + '\n'
                if 'example' in definition: description += 'Example: ' + str(definition['example']) + '\n' 
    return word, description


def google_search(content):
    url = google_search_url + content.replace(" ", "+") 
    #x = requests.get(google_search_url + query)
    return url


#print(x.text)
if __name__=='__main__':
    #a, b = search('freedom')
    #print('Word', a)
    #print('Description', b)
    print(google_search("How to create a whole new Pikachu?? !!!"))
