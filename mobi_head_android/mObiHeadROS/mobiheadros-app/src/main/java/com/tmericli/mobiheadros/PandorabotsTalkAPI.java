package com.tmericli.mobiheadros;

import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URI;
import java.net.URLEncoder;

/**
 * PandorabotsTalkAPI class allows conversation with a pandora chatbot
 *
 * @author Tekin Mericli
 * @version 1.0 2015
 */
public class PandorabotsTalkAPI {
    public String defaultCustid = "0";
    public String custid = defaultCustid;
    public String responseFailed = "RESPONSE FAILED";
    public String defaultBotId = "f5d922d97e345aa1";
    public String defaultHost = "www.pandorabots.com";

    public String askPandorabots(String input) {
        return askPandorabots(input, defaultHost, defaultBotId);
    }

    public String askPandorabots(String input, String host, String botid) {
        //System.out.println("Entering askPandorabots with input="+input+" host ="+host+" botid="+botid);
        String responseContent = pandorabotsRequest(input, host, botid);

        if (responseContent == null)
            return responseFailed;
        else
            return pandorabotsResponse(responseContent, host, botid);
    }

    public String responseContent(String url) throws Exception {
        HttpClient client = new DefaultHttpClient();
        HttpGet request = new HttpGet();
        request.setURI(new URI(url));
        InputStream is = client.execute(request).getEntity().getContent();
        BufferedReader inb = new BufferedReader(new InputStreamReader(is));
        StringBuilder sb = new StringBuilder("");
        String line;
        String NL = System.getProperty("line.separator");

        while ((line = inb.readLine()) != null) {
            sb.append(line).append(NL);
        }

        inb.close();

        return sb.toString();
    }


    public String spec(String host, String botid, String custid, String input) {
        //System.out.println("--> custid = "+custid);
        String spec = "";
        try {
            if (custid.equals("0"))      // get custid on first transaction with Pandorabots
                spec =    String.format("%s?botid=%s&input=%s",
                        "http://" + host + "/pandora/talk-xml",
                        botid,
                        URLEncoder.encode(input, "UTF-8"));
            else spec =                 // re-use custid on each subsequent interaction
                    String.format("%s?botid=%s&custid=%s&input=%s",
                            "http://" + host + "/pandora/talk-xml",
                            botid,
                            custid,
                            URLEncoder.encode(input, "UTF-8"));
        } catch (Exception ex) {
            ex.printStackTrace();
        }

        //System.out.println(spec);
        return spec;
    }

    public String pandorabotsRequest(String input, String host, String botid) {
        try {

            String spec = spec(host, botid, custid, input);
            //System.out.println("Spec = "+spec);
            String responseContent = responseContent(spec);

            return responseContent;
        } catch (Exception ex) {
            ex.printStackTrace();

            return null;
        }
    }
    public String pandorabotsResponse (String xmlRpcResponse, String host, String botid) {
        String botResponse = responseFailed;
        try {
            int n1 = xmlRpcResponse.indexOf("<that>");
            int n2 = xmlRpcResponse.indexOf("</that>");

            if (n2 > n1)
                botResponse = xmlRpcResponse.substring(n1+"<that>".length(), n2);

            n1 = xmlRpcResponse.indexOf("custid=");

            if (n1 > 0) {
                custid = xmlRpcResponse.substring(n1+"custid=\"".length(), xmlRpcResponse.length());
                n2 = custid.indexOf("\"");

                if (n2 > 0)
                    custid = custid.substring(0, n2);
                else
                    custid = defaultCustid;
            }
            if (botResponse.endsWith("."))
                botResponse = botResponse.substring(0, botResponse.length()-1);   // annoying Pandorabots extra "."

        } catch (Exception ex) {
            ex.printStackTrace();
        }

        return botResponse;
    }

}
