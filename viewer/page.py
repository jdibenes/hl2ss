import os
import xml.etree.ElementTree as ET 

class PageItem:
    def __init__(self, item_type, content, xmin, xmax, ymin, ymax):
        self.item_type = item_type
        self.content = content
        self.position = [xmin, xmax, ymin, ymax]

    def is_touched(self, touch_position):
        is_overlapped = False
        if touch_position[0] >= self.position[0] and touch_position[0] <= self.position[1] and touch_position[1] >= self.position[2] and touch_position[1] <= self.position[3]:
            is_overlapped = True
        return is_overlapped

    def get_content(self):
        return self.item_type, self.content
    
    def to_dict(self, with_content=False):
        dic = {
            "item_type": self.item_type,
            "pos": self.position
                }
        if with_content:
            dic["content"] = self.content
        return dic

class Page:
    def __init__(self, xml_path, file_path):
        self.xml_path = xml_path
        self.file_path = file_path
        self.items = []      
        self.parsing_page_content() 
    
    def get_item_dict(self, with_content=False):
        list_items = []
        for item in self.items:
            list_items.append(item.to_dict(with_content))
        return list_items

    def get_content_at_touch_position(self, touch_position):
        # TODO: using position to receive the 
        for i in range(len(self.items)):
            if self.items[i].is_touched(touch_position):
                a, b = self.items[i].get_content()
                return a, b
        return -1, -1

    def parsing_page_content(self):
        tree = ET.parse(self.xml_path)
        root = tree.getroot()
        for obj in root:
            xmin = 0
            xmax = 0
            ymin = 0
            ymax = 0
            item_type = ''
            content = 'test'
            if obj.tag == 'size':
                for child in obj:
                    if child.tag == 'width':
                        self.width = int(child.text)
                        #print(self.width)
                    elif child.tag == 'height':
                        self.height = int(child.text)
                        #print(self.height)
            if obj.tag == 'object':
                for child in obj:
                    if child.tag == 'name':
                        if 'text' in child.text:
                            item_type = 'text'
                        elif 'img' in child.text:
                            item_type = 'img'
                        elif 'ref_num' in child.text:
                            item_type = 'ref_num'
                    elif child.tag == 'content':
                        content = child.text
                    elif child.tag == 'bndbox':
                        for vertex in child:
                            if vertex.tag == 'xmin':
                                xmin = int(vertex.text)
                            elif vertex.tag == 'xmax':
                                xmax = int(vertex.text)
                            elif vertex.tag == 'ymin':
                                ymin = int(vertex.text)
                            elif vertex.tag == 'ymax':
                                ymax = int(vertex.text)
                #print(item_type, content, xmin, xmax, ymin, ymax)
                self.items.append(PageItem(item_type, content, xmin, xmax, ymin, ymax))

class Book:
    # page format "page (<number>).jpg"  number starts from 1
    def __init__(self, book_folder):
        self.no_pages = 0
        self.book_folder = book_folder
        for f in os.listdir(book_folder):
            if 'png' in f or 'jpg' in f:
                self.no_pages += 1

       
        self.pages = [-1]*self.no_pages
        
        

    def get_page(self, index):
        if (self.pages[index] == -1 or self.pages[index] ==-2):
            xml_path = os.path.join(self.book_folder, "page (%s).xml" % str(index + 1))
            file_path = os.path.join(self.book_folder, "page (%s).jpg" % str(index + 1))
            if os.path.isfile(xml_path):
                self.pages[index] = Page(xml_path, file_path)
            else:
                self.pages[index] = -2
                return -2
        return self.pages[index]

if __name__ == "__main__":
    book = Book('./books/opd/') 
    book.get_page(205)
    book.pages[205].parsing_page_content()
