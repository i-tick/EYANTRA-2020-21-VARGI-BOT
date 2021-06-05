/*This appscript is used to populate aur dashboard spreadsheet and website
also this appscript contains the necessary logic for sending email alerts*/

function doGet(e) {

    var ss = SpreadsheetApp.getActive();

    var sheet = ss.getSheetByName(e.parameter["id"]);

    var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];

    var lastRow = sheet.getLastRow();

    var cell = sheet.getRange('a1');
    var col = 0;
    var d = new Date();

    for (i in headers) {

        // loop through the headers and if a parameter name matches the header name insert the value

        if (headers[i] == "Timestamp") {
            val = d.toDateString() + ", " + d.toLocaleTimeString();
        }
        else {
            val = e.parameter[headers[i]];
        }

        // append data to the last row
        cell.offset(lastRow, col).setValue(val);
        col++;
    }
    //var sh = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
    var lastRow = sheet.getLastRow();
    var lastCol = sheet.getLastColumn();
    //var data = sh.getRange(lastRow, lastCol).getValue();
    var data = sheet.getDataRange().getValues();
    var xy = sheet.getRange(lastRow, lastCol).getValue();
    Logger.log("hi")
    mail_row = ""
    data.forEach(function (row) {
        //Logger.log(row);
        mail_row = row

    });
    var ldapIndex_1 = headers.indexOf('Order Dispatched');
    var dis_time = headers.indexOf('Dispatch Time');
    ++dis_time
    var x = mail_row[dis_time]
    var ldapIndex_2 = headers.indexOf('Order Shipped');

    if (mail_row[ldapIndex_1] == "YES" && mail_row[ldapIndex_2] == "NO") {

        //match and update the corresponding row
        var ldapIndex = headers.indexOf('Order Id');
        var statusIndex = headers.indexOf('Order Dispatched');
        var dispatch_time = headers.indexOf('Dispatch Time');
        var sheetRow;
        for (var i = 1; i < data.length; i++) {
            var row = data[i];
            if (row[ldapIndex] == mail_row[ldapIndex]) {
                // You have found the correct row, set + 1 because Sheet range starts from 1, not 0
                sheetRow = i + 1;
                // We have found the row, no need to iterate further
                break;
            }
        }
        // Also set statusIndex +1, because index in array is -1 compared to index in sheet
        ++statusIndex;
        //Set the value
        //await sheet.getRange(sheetRow, statusIndex ).setValue('YES');
        //await sheet.getRange(sheetRow, dis_time ).setValue(x);
        //var lastRow = sheet.getLastRow();
        --sheetRow;

        var cell = sheet.getRange('a1');
        var col = 0;
        var d = new Date();

        for (i in headers) {

            // loop through the headers and if a parameter name matches the header name insert the value

            if (headers[i] == "Timestamp") {
                val = d.toDateString() + ", " + d.toLocaleTimeString();
            }
            else {
                val = e.parameter[headers[i]];
            }

            // append data to the last row
            cell.offset(sheetRow, col).setValue(val);
            col++;
        }
        var to = "eyrc.vb.0566@gmail.com";   //write your email id here
        var message = "Hello !" + "\n" + "Your Order has been dispatched , contact us if you have any questions . " + "\n" + "We are here to help you" + "\n" + "\n" + "ORDER SUMMARY" + "\n\n" + "Order Number : " + mail_row[2] + "\n" + "Item : " + mail_row[3] + "\n" + "Quantity :" + mail_row[5] + "\n" + "Dispatch Date and Time :" + mail_row[12] + "\n" + "City : " + mail_row[6] + "\n" + "Cost : " + mail_row[15] + "\n" + mail_row[13]

        MailApp.sendEmail(to, " Your Order is Dispatched! - VB#0566", message);
        MailApp.sendEmail("eyrc.vb.0000@gmail.com", " Your Order is Dispatched! - VB#0566", message);



        var del_row=sheet.getLastRow()
        sheet.deleteRow(del_row); 
    }



    else if (mail_row[ldapIndex_2] == "YES") {

        //match and update the corresponding row
        var ldapIndex = headers.indexOf('Order Id');
        var statusIndex = headers.indexOf('Order Dispatched');
        var dispatch_time = headers.indexOf('Dispatch Time');
        var sheetRow;
        for (var i = 1; i < data.length; i++) {
            var row = data[i];
            var x = 0;
            if (row[ldapIndex] == mail_row[ldapIndex]) {
                x = 1;
                // You have found the correct row, set + 1 because Sheet range starts from 1, not 0
                sheetRow = i + 1;
                // We have found the row, no need to iterate further
                break;
            }
        }
        // Also set statusIndex +1, because index in array is -1 compared to index in sheet
        ++statusIndex;
        //Set the value
        //await sheet.getRange(sheetRow, statusIndex ).setValue('YES');
        //await sheet.getRange(sheetRow, dis_time ).setValue(x);
        //var lastRow = sheet.getLastRow();
        --sheetRow;

        var cell = sheet.getRange('a1');
        var col = 0;
        var d = new Date();

        for (i in headers) {

            // loop through the headers and if a parameter name matches the header name insert the value

            if (headers[i] == "Timestamp") {
                val = d.toDateString() + ", " + d.toLocaleTimeString();
            }
            else {
                val = e.parameter[headers[i]];
            }

            // append data to the last row
            cell.offset(sheetRow, col).setValue(val);
            
            col++;

        }
        var to = "eyrc.vb.0566@gmail.com";   //write your email id here
        var message = "Hello !" + "\n" + "Your Order has been shipped.It will be drone delivered to you in 1 day " + "\n" + "Contact us if you have any questions . " + "\n" + "We are here to help you" + "\n" + "\n" + "ORDER SUMMARY" + "\n\n" + "Order Number : " + mail_row[2] + "\n" + "Item : " + mail_row[3] + "\n" + "Quantity :" + mail_row[5] + "\n" + "Shipped Date and Time :" + mail_row[13] + "\n" + "City : " + mail_row[6] + "\n" + "Cost : " + mail_row[15] + "\n" + "Estimated Time of delivery : " + mail_row[16];

        MailApp.sendEmail(to, " Your Order is Shipped! - VB#0566", message);
        MailApp.sendEmail("eyrc.vb.0000@gmail.com", " Your Order is Shipped! - VB#0566", message);

        var del_row = sheet.getLastRow()
        sheet.deleteRow(del_row);
    }









    return ContentService.createTextOutput("Success");
}
