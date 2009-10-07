///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2005 University of Edinburgh (Michael White)
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//////////////////////////////////////////////////////////////////////////////

package opennlp.ccg.util;

import java.util.*;

/**
 * Implements a trie with a data object at each node.
*  Keys are assumed to be canonical, and thus checked using identity (==) 
 * rather than equality.  For efficient allocation, all children 
 * can be added at once.
 *
 * @author      Michael White
 * @version     $Revision: 1.2 $, $Date: 2005/10/13 20:33:49 $
 */
public class TrieMap<KeyType,DataType> {
    
    /** The data object. */
    public DataType data;
    
    /** The mapping to the children.  If there is just one child, it's 
        stored in a pair with its key.  Otherwise, an IdentityHashMap 
        is used. */
    private Object childMap = null;
    
    
    /** Constructor with data object. */
    public TrieMap(DataType data) { this.data = data; }
    
    
    /** Adds the given child with its key. */
    @SuppressWarnings("unchecked")
	public void addChild(KeyType key, TrieMap<KeyType,DataType> child) {
        if (childMap == null) { 
            childMap = new Pair<KeyType,TrieMap<KeyType,DataType>>(key, child);
            return;
        }
        Map<KeyType,TrieMap<KeyType,DataType>> map;
        if (childMap instanceof Pair) {
            Pair<KeyType,TrieMap<KeyType,DataType>> pair = (Pair) childMap;
            map = new IdentityHashMap<KeyType,TrieMap<KeyType,DataType>>();
            map.put(pair.a, pair.b);
            childMap = map;
        }
        else {
            map = (Map) childMap;
        }
        map.put(key, child);
    }
    
    /** Adds the given children with their keys. */
    @SuppressWarnings("unchecked")
	public void addChildren(List<KeyType> keys, List<TrieMap<KeyType,DataType>> childNodes) {
        if (childMap == null && keys.size() == 1) { 
        	TrieMap<KeyType,DataType> child = childNodes.get(0);
            childMap = new Pair<KeyType,TrieMap<KeyType,DataType>>(keys.get(0), child);
            return;
        }
        Map<KeyType,TrieMap<KeyType,DataType>> map;
        if (childMap == null) {
            map = new IdentityHashMap<KeyType,TrieMap<KeyType,DataType>>(keys.size());
            childMap = map;
        }
        else if (childMap instanceof Pair) {
            Pair<KeyType,TrieMap<KeyType,DataType>> pair = (Pair) childMap;
            map = new IdentityHashMap<KeyType,TrieMap<KeyType,DataType>>(keys.size()+1);
            map.put(pair.a, pair.b);
            childMap = map;
        }
        else {
            map = (Map) childMap;
        }
        for (int i = 0; i < keys.size(); i++) {
            TrieMap<KeyType,DataType> child = childNodes.get(i);
            map.put(keys.get(i), child);
        }
    }
    
    
    /** Gets the child for the given key, or null if none. */
    @SuppressWarnings("unchecked")
	public TrieMap<KeyType,DataType> getChild(KeyType key) {
        if (childMap == null) return null;
        if (childMap instanceof Pair) {
            Pair<KeyType,TrieMap<KeyType,DataType>> pair = (Pair) childMap;
            if (pair.a == key) return pair.b;
            else return null;
        }
        Map<KeyType,TrieMap<KeyType,DataType>> map = (Map) childMap;
        return map.get(key);
    }
    
    /** Gets the child for the given list of keys, or null if none. */
    public TrieMap<KeyType,DataType> getChildFromList(List<KeyType> keys) {
        TrieMap<KeyType,DataType> next = this;
        for (int pos = 0; pos < keys.size(); pos++) {
            next = next.getChild(keys.get(pos));
            if (next == null) return null;
        }
        return next;
    }
    
    
    /** Finds the child for the given key, 
        adding one (with a null data object) if necessary. */
    public TrieMap<KeyType,DataType> findChild(KeyType key) {
        TrieMap<KeyType,DataType> child = getChild(key);
        if (child == null) {
            child = new TrieMap<KeyType,DataType>(null);
            addChild(key, child);
        }
        return child;
    }
    
    /** Finds the child for the given list of keys, 
        adding one (with a null data object) if necessary, 
        along with any necessary intervening parents. */
    public TrieMap<KeyType,DataType> findChildFromList(List<KeyType> keys) {
        TrieMap<KeyType,DataType> next = this;
        for (int pos = 0; pos < keys.size(); pos++) {
            KeyType key = keys.get(pos);
            TrieMap<KeyType,DataType> child = next.getChild(key);
            if (child == null) {
                child = new TrieMap<KeyType,DataType>(null);
                next.addChild(key, child);
            }
            next = child;
        }
        return next;
    }
    
    /** Returns this trie map as a string, with indenting. */
    public String toString() {
        StringBuffer sb = new StringBuffer();
        toString(sb, "");
        return sb.toString();
    }
    
    // appends this trie map as a string, with the given indenting level, 
    // to the given string buffer
    @SuppressWarnings("unchecked")
	private void toString(StringBuffer sb, String indent) { 
        sb.append("node: " + data);
        if (childMap == null) return;
        indent += "  ";
        if (childMap instanceof Pair) {
            Pair<KeyType,TrieMap<KeyType,DataType>> pair = (Pair) childMap;
            toString(sb, indent, pair.a, pair.b);
        }
        else {
            Map<KeyType,TrieMap<KeyType,DataType>> map = (Map) childMap;
            List<KeyType> keys = new ArrayList<KeyType>(map.keySet());
            Comparator<KeyType> toStringComparator = new Comparator<KeyType>() {
                public int compare(KeyType o1, KeyType o2) {
                    return o1.toString().compareTo(o2.toString());
                }
            };
            Collections.sort(keys, toStringComparator);
            for (KeyType key : keys) { 
                toString(sb, indent, key, map.get(key));
            }
        }
    }
    
    // appends the given key and child
    private void toString(StringBuffer sb, String indent, Object key, TrieMap child) {
        sb.append("\n").append(indent).append('[').append(key).append("] ");
        child.toString(sb, indent);
    }
}
