--TODO: check if you can use lua type boolean instead of strings and pass that back to osm2pgsql
--with the hopes that they will become strings once they get back to c++ and then just work in
--postgres

drive_on_right = {
["Anguilla"] = "false",
["Antigua and Barbuda"] = "false",
["Australia"] = "false",
["Bangladesh"] = "false",
["Barbados"] = "false",
["Bermuda"] = "false",
["Bhutan"] = "false",
["Botswana"] = "false",
["British Virgin Islands"] = "false",
["Brunei Darussalam"] = "false",
["Cayman Islands"] = "false",
["Cook Islands"] = "false",
["Cyprus"] = "false",
["Dominica"] = "false",
["East Timor"] = "false",
["England"] = "false",
["Falkland Islands"] = "false",
["Grenada"] = "false",
["Guernsey"] = "false",
["Guyana"] = "false",
["Hong Kong"] = "false",
["India"] = "false",
["Indonesia"] = "false",
["Ireland"] = "false",
["Isle of Man"] = "false",
["Jamaica"] = "false",
["Japan"] = "false",
["Jersey"] = "false",
["Kenya"] = "false",
["Kiribati"] = "false",
["Lesotho"] = "false",
["Macao"] = "false",
["Malawi"] = "false",
["Malaysia"] = "false",
["Maldives"] = "false",
["Malta"] = "false",
["Mauritius"] = "false",
["Moçambique"] = "false",
["Montserrat"] = "false",
["Namibia"] = "false",
["Naoero"] = "false",
["Nepal"] = "false",
["New Zealand"] = "false",
["Niue"] = "false",
["Northern Ireland"] = "false",
["Pakistan"] = "false",
["Papua Niugini"] = "false",
["Pitcairn Islands"] = "false",
["Republic of Ireland"] = "false",
["Saint Helena, Ascension and Tristan da Cunha"] = "false",
["Saint Kitts and Nevis"] = "false",
["Saint Lucia"] = "false",
["Saint Vincent and the Grenadines"] = "false",
["Samoa"] = "false",
["Sesel"] = "false",
["Singapore"] = "false",
["Solomon Islands"] = "false",
["Soomaaliya"] = "false",
["South Africa"] = "false",
["Sri Lanka"] = "false",
["Suriname"] = "false",
["Alba / Scotland"] = "false",
["Swatini"] = "false",
["Tanzania"] = "false",
["Thailand"] = "false",
["The Bahamas"] = "false",
["Tokelau"] = "false",
["Tonga"] = "false",
["Trinidad and Tobago"] = "false",
["Turks and Caicos Islands"] = "false",
["Tuvalu"] = "false",
["Uganda"] = "false",
["United Kingdom"] = "false",
["United States Virgin Islands"] = "false",
["Viti"] = "false",
["Cymru / Wales"] = "false",
["Zambia"] = "false",
["Zimbabwe"] = "false"
}

allow_intersection_names = {
["Japan"] = "true",
["North Korea"] = "true",
["South Korea"] = "true",
["Nicaragua"] = "true"
}

--returns 1 if you should filter this way 0 otherwise
function filter_tags_generic(kv)
--  if (kv["boundary"] == "administrative" and
--     (kv["admin_level"] == "2" or kv["admin_level"] == "4")) then

     delete_tags = { 'FIXME', 'note', 'source' }

     for i,k in ipairs(delete_tags) do
        kv[k] = nil
     end

     return 0
--  end

--  return 1
end

function nodes_proc (kv, nokeys)
  return 0, kv
end

function ways_proc (kv, nokeys)
  --if there were no tags passed in, ie keyvalues is empty
  if nokeys == 0 then
    return 1, kv, 0, 0
  end

  --does it at least have some interesting tags
  filter = filter_tags_generic(kv)

  --let the caller know if its a keeper or not and give back the  modified tags
  --also tell it whether or not its a polygon or road
  return filter, kv, 0, 0
end

-- We save admins as 2(country) or 4(state/prov).
-- Geofabrik Europe extracts omit overseas / Asian members of some admin_level=2
-- "empire" relations, so valhalla_build_admins marks them degenerate and skips them
-- (ES/NL/NO/RU historically missing). Mirror the France workaround: drop the broken
-- level-2 relation and promote a mainland-complete stand-in with a hard-coded ISO.
function rels_proc (kv, nokeys)

  if (kv["type"] == "boundary" and kv["default_language"] and
     (((kv["boundary"] == "administrative" or kv["boundary"] == "territorial") and (kv["admin_level"] and tonumber(kv["admin_level"]) > 4)) or
     (kv["boundary"] == "political" or kv["political_division"] == "linguistic_community"))) then

     kv["iso_code"] = nil
     kv["admin_level"] = kv["admin_level"] or "15" --assign a high admin level for linguistic_community
     kv["drive_on_right"] = "false"
     kv["allow_intersection_names"] = "false"

     delete_tags = { 'FIXME', 'note', 'source' }

     for i,k in ipairs(delete_tags) do
        kv[k] = nil
     end

     return 0, kv
  end

  -- Spain: Europe/Madrid timezone covers peninsular Spain + Balearics (not Canarias).
  if kv["type"] == "boundary" and kv["boundary"] == "timezone" and
     (kv["timezone"] == "Europe/Madrid" or kv["name"] == "Zona Horaria de Europa/Madrid" or
      kv["name:en"] == "Europe/Madrid Timezone") then
     kv["boundary"] = "administrative"
     kv["admin_level"] = "2"
     kv["iso_code"] = "ES"
     kv["name"] = "Spain"
     kv["name:en"] = "Spain"
     kv["drive_on_right"] = "true"
     kv["allow_intersection_names"] = "false"
     for _, k in ipairs({ 'FIXME', 'note', 'source' }) do kv[k] = nil end
     return 0, kv
  end

  -- Netherlands: legal "Europees Nederland" polygon (Kingdom level-2 + land level-3
  -- both still tend to miss Caribbean members / fail geometry on Europe extracts).
  if kv["type"] == "boundary" and kv["boundary"] == "legal" and
     (kv["name:en"] == "European Netherlands" or kv["name"] == "Europees Nederland" or
      kv["timezone"] == "Europe/Amsterdam") then
     kv["boundary"] = "administrative"
     kv["admin_level"] = "2"
     kv["iso_code"] = "NL"
     kv["name"] = "Netherlands"
     kv["name:en"] = "Netherlands"
     kv["drive_on_right"] = "true"
     kv["allow_intersection_names"] = "false"
     for _, k in ipairs({ 'FIXME', 'note', 'source' }) do kv[k] = nil end
     return 0, kv
  end

  -- Russia: Europe/Moscow timezone covers most of European Russia (full RU level-2
  -- and federal districts are incomplete in Geofabrik Europe extracts).
  if kv["type"] == "boundary" and kv["boundary"] == "timezone" and
     (kv["timezone"] == "Europe/Moscow" or kv["name:en"] == "Moscow Time" or
      kv["name"] == "Московское время") then
     kv["boundary"] = "administrative"
     kv["admin_level"] = "2"
     kv["iso_code"] = "RU"
     kv["name"] = "Russia"
     kv["name:en"] = "Russia"
     kv["drive_on_right"] = "true"
     kv["allow_intersection_names"] = "false"
     for _, k in ipairs({ 'FIXME', 'note', 'source' }) do kv[k] = nil end
     return 0, kv
  end

  -- Mainland Norway land boundary (OSM r1059668): administrative, but no admin_level tag.
  -- Official level-2 Norway also pulls in Svalbard / Bouvet / Jan Mayen.
  if kv["type"] == "boundary" and kv["boundary"] == "administrative" and
     (kv["admin_level"] == nil or kv["admin_level"] == "") and
     (kv["name:en"] == "Norway" or kv["name"] == "Norge") then
     kv["admin_level"] = "2"
     kv["iso_code"] = "NO"
     kv["name"] = "Norway"
     kv["name:en"] = "Norway"
     kv["drive_on_right"] = "true"
     kv["allow_intersection_names"] = "false"
     for _, k in ipairs({ 'FIXME', 'note', 'source' }) do kv[k] = nil end
     return 0, kv
  end

  if (kv["type"] == "boundary" and (kv["boundary"] == "administrative" or kv["boundary"] == "territorial") and
     (kv["admin_level"] == "2" or kv["admin_level"] == "3" or kv["admin_level"] == "4" or kv["admin_level"] == "6")) then

     -- Keep only selected admin_level=3 areas (promoted to country below).
     if (kv["admin_level"] == "3") then
        local keep_l3 =
          kv["name"] == "Guyane" or kv["name"] == "Guadeloupe" or kv["name"] == "La Réunion" or
          kv["name"] == "Martinique" or kv["name"] == "Mayotte" or kv["name"] == "Saint-Pierre-et-Miquelon" or
          kv["name"] == "Saint-Barthélemy" or kv["name"] == "Saint-Martin (France)" or
          kv["name"] == "Polynésie Française" or kv["name"] == "Wallis-et-Futuna" or
          kv["name"] == "Nouvelle-Calédonie" or kv["name"] == "Île de Clipperton" or
          kv["name"] == "Terres australes et antarctiques françaises" or
          kv["name:en"] == "Metropolitan France" or
          kv["name:en"] == "Hong Kong" or kv["name"] == "Metro Manila" or
          -- European Netherlands (land) inside the Kingdom relation
          kv["name:en"] == "Netherlands" or kv["name"] == "Nederland" or
          -- European Russia federal districts (full RU level-2 is not in Europe extract)
          kv["name:en"] == "Central Federal District" or
          kv["name:en"] == "Northwestern Federal District" or
          kv["name:en"] == "Southern Federal District" or
          kv["name:en"] == "North Caucasian Federal District" or
          kv["name:en"] == "Volga Federal District"
        if not keep_l3 then
          return 1, kv
        end
        -- French overseas / metro default language only
        if kv["default_language"] == nil and
           (kv["name:en"] == "Metropolitan France" or kv["name"] == "Guyane" or
            kv["name"] == "Guadeloupe" or kv["name"] == "La Réunion" or
            kv["name"] == "Martinique" or kv["name"] == "Mayotte" or
            kv["name"] == "Saint-Pierre-et-Miquelon" or kv["name"] == "Saint-Barthélemy" or
            kv["name"] == "Saint-Martin (France)" or kv["name"] == "Polynésie Française" or
            kv["name"] == "Wallis-et-Futuna" or kv["name"] == "Nouvelle-Calédonie" or
            kv["name"] == "Île de Clipperton" or
            kv["name"] == "Terres australes et antarctiques françaises") then
          kv["default_language"] = "fr"
        end
     end

     if kv["admin_level"] == "6" and kv["name"] ~= "District of Columbia" then
       return 1, kv
     end

     -- Drop incomplete empire-style level-2 country relations (Geofabrik Europe extract).
     if kv["admin_level"] == "2" then
        if kv["name"] == "France" or
           kv["name:en"] == "Spain" or kv["name"] == "España" or
           kv["name:en"] == "Netherlands" or kv["name"] == "Nederland" or
           kv["name:en"] == "Norway" or kv["name"] == "Norge" or
           kv["name:en"] == "Russia" or kv["name"] == "Россия" then
          return 1, kv
        elseif kv["name:en"] == "Abkhazia" or kv["name:en"] == "South Ossetia" then
          kv["admin_level"] = "4"
        end
     end

     if kv["name"] == "Metro Manila" then
        kv["admin_level"] = "4"
     end

     if kv["admin_level"] == "3" then
       kv["admin_level"] = "2"
       if kv["name:en"] == "Metropolitan France" then
         kv["name"] = "France"
         kv["iso_code"] = "FR"
       elseif kv["name:en"] == "Netherlands" or kv["name"] == "Nederland" then
         kv["name"] = "Netherlands"
         kv["name:en"] = "Netherlands"
         kv["iso_code"] = "NL"
       elseif kv["name:en"] == "Central Federal District" or
              kv["name:en"] == "Northwestern Federal District" or
              kv["name:en"] == "Southern Federal District" or
              kv["name:en"] == "North Caucasian Federal District" or
              kv["name:en"] == "Volga Federal District" then
         kv["name"] = "Russia"
         kv["name:en"] = "Russia"
         kv["iso_code"] = "RU"
       end
     end

     if kv["admin_level"] == "6" then
       kv["admin_level"] = "4"
     end

     if kv["admin_level"] == "2" then
       if kv["iso_code"] == nil then
         if kv["ISO3166-1:alpha2"] then
           kv["iso_code"] = kv["ISO3166-1:alpha2"]
         elseif kv["ISO3166-1"] then
           kv["iso_code"] = kv["ISO3166-1"]
         end
       end
       if kv["name"] == "British Sovereign Base Areas" and kv["iso_code"] == nil then
         kv["iso_code"] = "GB"
       end
     elseif kv["admin_level"] == "4" then
       if kv["ISO3166-2"] then
         i, j = string.find(kv["ISO3166-2"], '-', 1, true)
         if i == 3 then
           if string.len(kv["ISO3166-2"]) == 6 or string.len(kv["ISO3166-2"]) == 5 then
             kv["iso_code"] = string.sub(kv["ISO3166-2"], 4)
           end
         elseif string.find(kv["ISO3166-2"], '-', 1, true) == nil then
           if string.len(kv["ISO3166-2"]) == 2 or  string.len(kv["ISO3166-2"]) == 3 then 
             kv["iso_code"] = kv["ISO3166-2"]
           elseif string.len(kv["ISO3166-2"]) == 4 or  string.len(kv["ISO3166-2"]) == 5 then
             kv["iso_code"] = string.sub(kv["ISO3166-2"], 3)
           end
         end
       end
     end

     kv["drive_on_right"] = drive_on_right[kv["name"]] or drive_on_right[kv["name:en"]] or "true"
     kv["allow_intersection_names"] = allow_intersection_names[kv["name"]] or allow_intersection_names[kv["name:en"]] or "false"

     delete_tags = { 'FIXME', 'note', 'source' }

     for i,k in ipairs(delete_tags) do
        kv[k] = nil
     end

     return 0, kv
  end

  return 1, kv
end

function rel_members_proc (keyvalues, keyvaluemembers, roles, membercount)
  --because we filter all rels we never call this function
  --because we do rel processing later we simply say that no ways are used
  --in the given relation, what would be nice is if we could push tags
  --back to the ways via keyvaluemembers, we could then avoid doing
  --post processing to get the shielding and directional highway info
  membersuperseeded = {}
  for i = 1, membercount do
    membersuperseeded[i] = 0
  end

  return 1, keyvalues, membersuperseeded, 0, 0, 0
end


