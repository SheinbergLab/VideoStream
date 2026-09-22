// Generic GenICam node access shared by the camera sources. Included inside
// a namespace that has already done
//     using namespace <vendor>::GenApi;  using namespace <vendor>::GenICam;
// (Spinnaker nests the GenICam reference implementation under Spinnaker::,
// Lucid ships it at global scope) so the same code serves both SDKs. Only
// the standard GenApi interfaces are used.

static const char* genicamAccessName(EAccessMode m) {
  switch (m) {
    case NI: return "NI";
    case NA: return "NA";
    case WO: return "WO";
    case RO: return "RO";
    case RW: return "RW";
    default: return "?";
  }
}

// Fill info for the node `name`; false with `err` set if there is no such node
static bool genicamNodeInfo(INodeMap* nm, const std::string& name,
                            ICameraControl::NodeInfo& info, std::string& err) {
  info = ICameraControl::NodeInfo();
  info.name = name;
  try {
    INode* node = nm->GetNode(name.c_str());
    if (!node) {
      err = "no such node: " + name;
      return false;
    }
    info.access = genicamAccessName(node->GetAccessMode());
    try { info.description = node->GetToolTip().c_str(); } catch (...) {}

    switch (node->GetPrincipalInterfaceType()) {
      case intfIInteger: {
        info.type = "integer";
        CIntegerPtr p(node);
        if (IsReadable(p)) {
          info.value = std::to_string(p->GetValue());
          info.min = (double)p->GetMin();
          info.max = (double)p->GetMax();
          info.inc = (double)p->GetInc();
          info.has_range = true;
          try { info.unit = p->GetUnit().c_str(); } catch (...) {}
        }
        break;
      }
      case intfIFloat: {
        info.type = "float";
        CFloatPtr p(node);
        if (IsReadable(p)) {
          info.value = std::to_string(p->GetValue());
          info.min = p->GetMin();
          info.max = p->GetMax();
          try { if (p->HasInc()) info.inc = p->GetInc(); } catch (...) {}
          info.has_range = true;
          try { info.unit = p->GetUnit().c_str(); } catch (...) {}
        }
        break;
      }
      case intfIBoolean: {
        info.type = "boolean";
        CBooleanPtr p(node);
        if (IsReadable(p)) info.value = p->GetValue() ? "1" : "0";
        break;
      }
      case intfIEnumeration: {
        info.type = "enumeration";
        CEnumerationPtr p(node);
        if (IsReadable(p)) {
          CEnumEntryPtr cur = p->GetCurrentEntry();
          if (cur.IsValid()) info.value = cur->GetSymbolic().c_str();
        }
        NodeList_t entries;
        p->GetEntries(entries);
        for (size_t i = 0; i < entries.size(); i++) {
          CEnumEntryPtr e = entries[i];
          if (e.IsValid() && IsAvailable(e)) info.entries.push_back(e->GetSymbolic().c_str());
        }
        break;
      }
      case intfIString: {
        info.type = "string";
        CStringPtr p(node);
        if (IsReadable(p)) info.value = p->GetValue().c_str();
        break;
      }
      case intfICommand:
        info.type = "command";
        break;
      default: {
        info.type = "other";
        CValuePtr p(node);
        if (p.IsValid() && IsReadable(p)) {
          try { info.value = p->ToString().c_str(); } catch (...) {}
        }
        break;
      }
    }
    return true;
  } catch (std::exception& e) {
    err = name + ": " + e.what();
    return false;
  }
}

static bool genicamParseBool(const std::string& s, bool& out) {
  std::string v;
  for (char c : s) v += (char)tolower((unsigned char)c);
  if (v == "1" || v == "true" || v == "on" || v == "yes") { out = true; return true; }
  if (v == "0" || v == "false" || v == "off" || v == "no") { out = false; return true; }
  return false;
}

// Write `value` (string form, see ICameraControl) to node `name`
static bool genicamSetNode(INodeMap* nm, const std::string& name,
                           const std::string& value, std::string& err) {
  try {
    INode* node = nm->GetNode(name.c_str());
    if (!node) {
      err = "no such node: " + name;
      return false;
    }
    if (!IsWritable(node)) {
      err = name + " is not writable (" + genicamAccessName(node->GetAccessMode()) + ")";
      return false;
    }
    switch (node->GetPrincipalInterfaceType()) {
      case intfIInteger: {
        CIntegerPtr p(node);
        p->SetValue(std::stoll(value));
        return true;
      }
      case intfIFloat: {
        CFloatPtr p(node);
        p->SetValue(std::stod(value));
        return true;
      }
      case intfIBoolean: {
        bool b;
        if (!genicamParseBool(value, b)) {
          err = name + ": expected a boolean, got '" + value + "'";
          return false;
        }
        CBooleanPtr p(node);
        p->SetValue(b);
        return true;
      }
      case intfIEnumeration: {
        CEnumerationPtr p(node);
        CEnumEntryPtr e = p->GetEntryByName(value.c_str());
        if (!e.IsValid() || !IsAvailable(e)) {
          err = name + ": no entry '" + value + "' (choices:";
          NodeList_t entries;
          p->GetEntries(entries);
          for (size_t i = 0; i < entries.size(); i++) {
            CEnumEntryPtr ee = entries[i];
            if (ee.IsValid() && IsAvailable(ee)) err += std::string(" ") + ee->GetSymbolic().c_str();
          }
          err += ")";
          return false;
        }
        p->SetIntValue(e->GetValue());
        return true;
      }
      case intfIString: {
        CStringPtr p(node);
        p->SetValue(value.c_str());
        return true;
      }
      case intfICommand: {
        CCommandPtr p(node);
        p->Execute();
        return true;
      }
      default:
        err = name + ": unsupported node type";
        return false;
    }
  } catch (std::invalid_argument&) {
    err = name + ": bad value '" + value + "'";
    return false;
  } catch (std::exception& e) {
    err = name + ": " + e.what();
    return false;
  }
}

// Names of all feature nodes (categories excluded)
static void genicamListNodes(INodeMap* nm, std::vector<std::string>& names) {
  try {
    NodeList_t nodes;
    nm->GetNodes(nodes);
    for (size_t i = 0; i < nodes.size(); i++) {
      INode* n = nodes[i];
      if (!n) continue;
      EInterfaceType t = n->GetPrincipalInterfaceType();
      if (t == intfICategory || t == intfIEnumEntry || t == intfIPort || t == intfIRegister) continue;
      names.push_back(n->GetName().c_str());
    }
  } catch (...) {}
}
