//This appscript contains the logic to populate orders_shipped spreadsheet

function doGet(e){
  
  var ss = SpreadsheetApp.getActive();

  var sheet = ss.getSheetByName(e.parameter["id"]);

  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];

  var lastRow = sheet.getLastRow();

  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();

  for (i in headers){

    

    if (headers[i] == "Timestamp")
    {
      val = d.toDateString() + ", " + d.toLocaleTimeString();
    }
    else
    {
      val = e.parameter[headers[i]]; 
    }

    
    cell.offset(lastRow, col).setValue(val);
    col++;
  }

  var lastRow = sheet.getLastRow();
  var lastCol = sheet.getLastColumn();
  var data = sheet.getDataRange().getValues();
  var xy = sheet.getRange(lastRow,lastCol).getValue();
  Logger.log("hi")
  mail_row=""
  data.forEach(function (row) {
   mail_row=row
   
 });




  return ContentService.createTextOutput(mail_row);
}