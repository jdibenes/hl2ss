using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Serialization;
using UnityEngine;
using UnityEngine.Rendering.VirtualTexturing;
using UnityEngine.UI;

public class PageInformationHolder : MonoBehaviour
{
    public Material m_material;


    public int width_page;
    public int height_page;
    public int page_position = 0; // 1 = left 2 = right 
    public List<int> list_xmin;
    public List<int> list_xmax;
    public List<int> list_ymin;
    public List<int> list_ymax;

    public GameObject pageObject;
    public List <GameObject> vis_object;
    public List<string> list_item_type;

    public void Start()
    {
 /*     
        this.AddItem("ok", 498, 830, 372, 780);
        this.AddItem("ok", 346, 827, 81, 775);
        this.AddItem("ok", 238, 900, 82, 845);
        this.AddItem("ok", 354, 984, 59, 920);
        this.AddItem("ok", 780, 905, 377, 848);
        this.AddItem("ok", 765, 984, 379, 928);
        this.AddItem("ok", 1006, 834, 816, 773);
        this.AddItem("ok", 1098, 905, 813, 855);
        this.AddItem("ok", 1000, 981, 809, 931);
        this.AddItem("ok", 1338, 832, 1119, 787);
        this.AddItem("ok", 1363, 913, 1113, 853);
        this.AddItem("ok", 1357, 991, 1112, 930);
        this.AddItem("ok", 1623, 838, 1368, 780);
        this.AddItem("ok", 289, 1377, 39, 1081);
        this.AddItem("ok", 501, 1480, 297, 1237);
        this.AddItem("ok", 501, 1480, 297, 1237);
        this.AddItem("ok", 698, 1259, 493, 1031);
        this.AddItem("ok", 888, 1477, 698, 1216);
        this.AddItem("ok", 1115, 1353, 890, 1075);
        this.AddItem("ok", 1409, 1446, 1126, 1165);
        this.AddItem("ok", 1648, 1291, 1419, 1030);
        this.AddItem("ok", 261, 1880, 29, 1624);
        this.AddItem("ok", 484, 2007, 259, 1677);
        this.AddItem("ok", 713, 1853, 483, 1595);
        this.AddItem("ok", 980, 2007, 716, 1727);
        this.AddItem("ok", 980, 2007, 716, 1727);
        this.AddItem("ok", 1227, 1904, 983, 1593);
        this.AddItem("ok", 1227, 1904, 983, 1593);
        this.AddItem("ok", 1405, 2002, 1234, 1742);
        this.AddItem("ok", 1634, 1856, 1405, 1628);

        this.width_page = 1673;
        this.height_page = 2112;
        // this.AddItem("ok",1,5,1,5);
        this.VisualizeAll();
 */

    }
    public uint AddItem(string item_type, int xmin, int xmax, int ymin, int ymax)
    {
        //int xmax, int ymax, int xmin, int ymin)
    //{
        //



        this.list_item_type.Add(item_type);
        this.list_xmin.Add(xmin);
        this.list_xmax.Add(xmax);
        this.list_ymin.Add(ymin);
        this.list_ymax.Add(ymax);

        //this.list_ymax=this.list_ymax.Append(ymax).ToList();

        return 0;
    }


    public int Visualize(int index) {
        float xmin = this.list_xmin[index];
        float xmax = this.list_xmax[index];
        float ymin = this.list_ymin[index];
        float ymax = this.list_ymax[index];

        float localWidth = (xmax - xmin) / (width_page * 1.0f);
        float localHeight = (ymax - ymin) / (height_page * 1.0f);

        float localCenterX = ((xmin + xmax) / (width_page * 2.0f)) - 0.5f;
        float localCenterY = - ((ymin + ymax) / (height_page * 2.0f)) + 0.5f;
   
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.SetParent(pageObject.transform);
        cube.transform.localPosition = new Vector3(localCenterX, localCenterY, 0f);
        cube.transform.localScale = new Vector3(localWidth, localHeight, 0.01f);
        cube.transform.localRotation = Quaternion.identity;
        cube.GetComponent<Renderer>().material = m_material;
        //Outline script = cube.AddComponent<Outline>();
        //script.OutlineMode = Outline.Mode.OutlineAll;
        //script.OutlineColor = Color.black;
        //script.OutlineWidth = 10;
        this.vis_object.Add(cube);

        return 0;
    }

    public int DestroyAllVisualizedObject() {
        for (int i = 0; i < this.vis_object.Count; ++i)
        {
            GameObject tmp = this.vis_object[i];
            Destroy(tmp);

        }

        vis_object.Clear();
        return 0;
    }
    public int VisualizeAll() {
        if (page_position == 2)
            pageObject = GameObject.FindWithTag("RightPlane");
        else
            pageObject = GameObject.FindWithTag("LeftPlane");
        Debug.Log("Count" + list_item_type.Count.ToString());
        for (int i = 0; i < list_item_type.Count; ++i) {
            Debug.Log("?"+i.ToString());
            this.Visualize(i);
       }
        
        return 0; 
    }

}